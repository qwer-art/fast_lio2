#include "spark_lio_sdk.h"

#include <iostream>

#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/storage_filter.hpp>
#include <rosbag2_storage/storage_options.hpp>

namespace spark_fast_lio {

// ------------------------------------------------------------------
// BagReader: wraps rosbag2_cpp::Reader, hides the header from sdk.h
// ------------------------------------------------------------------
struct SparkLioSdk::BagReader {
  rosbag2_cpp::Reader reader;
  rclcpp::Serialization<sensor_msgs::msg::PointCloud2> cloud_ser;
  rclcpp::Serialization<sensor_msgs::msg::Imu> imu_ser;
  std::string lidar_topic;
  std::string imu_topic;
  bool opened = false;
};

// ------------------------------------------------------------------
// Constructor / Destructor
// ------------------------------------------------------------------
SparkLioSdk::SparkLioSdk() : preprocessor_(std::make_shared<Preprocess>()) {}

SparkLioSdk::~SparkLioSdk() { close(); }

// ------------------------------------------------------------------
// open
// ------------------------------------------------------------------
bool SparkLioSdk::open(const std::string &bag_path,
                       const std::string &lidar_topic,
                       const std::string &imu_topic,
                       const Config &config) {
  verbose_ = config.verbose;

  // Configure preprocessor
  preprocessor_->lidar_type       = config.lidar_type;
  preprocessor_->N_SCANS          = config.scan_line;
  preprocessor_->SCAN_RATE        = config.scan_rate;
  preprocessor_->time_unit        = config.time_unit;
  preprocessor_->blind            = config.blind;
  preprocessor_->point_filter_num = config.point_filter_num;

  // Create and open bag reader
  bag_reader_ = std::make_unique<BagReader>();
  bag_reader_->lidar_topic = lidar_topic;
  bag_reader_->imu_topic   = imu_topic;

  try {
    rosbag2_storage::StorageOptions opts;
    opts.uri        = bag_path;
    opts.storage_id = "sqlite3";
    bag_reader_->reader.open(opts);
    bag_reader_->opened = true;
  } catch (const std::exception &e) {
    std::cerr << "[SparkLioSdk] Failed to open bag: " << e.what() << std::endl;
    return false;
  }

  // Print bag metadata
  auto metadata = bag_reader_->reader.get_metadata();
  std::cout << "[SparkLioSdk] Bag opened: " << bag_path << std::endl;
  std::cout << "[SparkLioSdk]   Message count: " << metadata.message_count << std::endl;
  std::cout << "[SparkLioSdk]   Topics:" << std::endl;
  for (auto &ti : metadata.topics_with_message_count) {
    std::cout << "[SparkLioSdk]     " << ti.topic_metadata.name
              << " (" << ti.topic_metadata.type
              << ", " << ti.message_count << " msgs)" << std::endl;
  }

  // Set topic filter for efficiency
  rosbag2_storage::StorageFilter filter;
  filter.topics = {lidar_topic, imu_topic};
  bag_reader_->reader.set_filter(filter);

  // Reset sync state
  lidar_pushed_        = false;
  lidar_end_time_      = 0.0;
  lidar_mean_scantime_ = 0.0;
  scan_num_            = 0;
  last_imu_time_       = 0.0;
  eof_                 = false;
  lidar_buffer_.clear();
  time_buffer_.clear();
  imu_buffer_.clear();

  return true;
}

// ------------------------------------------------------------------
// close
// ------------------------------------------------------------------
void SparkLioSdk::close() {
  if (bag_reader_ && bag_reader_->opened) {
    bag_reader_->reader.close();
    bag_reader_->opened = false;
  }
  bag_reader_.reset();
  eof_ = true;
}

bool SparkLioSdk::isEof() const { return eof_; }

// ------------------------------------------------------------------
// pushLidarMessage
// ------------------------------------------------------------------
void SparkLioSdk::pushLidarMessage(const sensor_msgs::msg::PointCloud2 &cloud_msg) {
  std::lock_guard<std::mutex> lk(buffer_mutex_);

  double msg_time = rclcpp::Time(cloud_msg.header.stamp).seconds();

  PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
  preprocessor_->process(cloud_msg, ptr);

  lidar_buffer_.push_back(ptr);
  time_buffer_.push_back(msg_time);
}

// ------------------------------------------------------------------
// pushImuMessage
// ------------------------------------------------------------------
void SparkLioSdk::pushImuMessage(const sensor_msgs::msg::Imu::ConstSharedPtr &imu_msg) {
  std::lock_guard<std::mutex> lk(buffer_mutex_);
  imu_buffer_.push_back(imu_msg);
  last_imu_time_ = rclcpp::Time(imu_msg->header.stamp).seconds();
}

// ------------------------------------------------------------------
// syncPackages (extracted from SPARKFastLIO2::syncPackages)
// ------------------------------------------------------------------
bool SparkLioSdk::syncPackages(MeasureGroup &meas) {
  std::lock_guard<std::mutex> lk(buffer_mutex_);

  if (lidar_buffer_.empty() || imu_buffer_.empty()) return false;

  if (!lidar_pushed_) {
    meas.lidar          = lidar_buffer_.front();
    meas.lidar_beg_time = time_buffer_.front();
    static double denominator = 1000;

    if (meas.lidar->points.size() <= 1) {
      lidar_end_time_ = meas.lidar_beg_time + lidar_mean_scantime_;
    } else if (meas.lidar->points.back().curvature / denominator < 0.5 * lidar_mean_scantime_) {
      lidar_end_time_ = meas.lidar_beg_time + lidar_mean_scantime_;
    } else {
      scan_num_++;
      if (meas.lidar->points.back().curvature < 80 ||
          meas.lidar->points.back().curvature > 120) {
        if (verbose_) {
          std::cerr << "[SparkLioSdk] WARNING: curvature (" << meas.lidar->points.back().curvature
                    << ") should be close to 100. Check timestamp_unit." << std::endl;
        }
      }

      double dt       = meas.lidar->points.back().curvature / 1000.0;
      lidar_end_time_ = meas.lidar_beg_time + dt;
      lidar_mean_scantime_ += (dt - lidar_mean_scantime_) / static_cast<double>(scan_num_);
    }
    meas.lidar_end_time = lidar_end_time_;
    lidar_pushed_       = true;
  }

  if (last_imu_time_ < lidar_end_time_) {
    return false;
  }

  /*** push imu data, and pop from imu buffer ***/
  double imu_time = rclcpp::Time(imu_buffer_.front()->header.stamp).seconds();
  meas.imu.clear();
  while ((!imu_buffer_.empty()) && (imu_time < lidar_end_time_)) {
    imu_time = rclcpp::Time(imu_buffer_.front()->header.stamp).seconds();
    if (imu_time > lidar_end_time_) break;
    meas.imu.push_back(imu_buffer_.front());
    imu_buffer_.pop_front();
  }

  lidar_buffer_.pop_front();
  time_buffer_.pop_front();
  lidar_pushed_ = false;

  return true;
}

// ------------------------------------------------------------------
// fillBuffersUntilSynced
// ------------------------------------------------------------------
bool SparkLioSdk::fillBuffersUntilSynced(MeasureGroup &meas) {
  while (bag_reader_->reader.has_next()) {
    auto bag_msg = bag_reader_->reader.read_next();
    rclcpp::SerializedMessage ser_msg(*bag_msg->serialized_data);

    if (bag_msg->topic_name == bag_reader_->lidar_topic) {
      sensor_msgs::msg::PointCloud2 cloud;
      bag_reader_->cloud_ser.deserialize_message(&ser_msg, &cloud);
      pushLidarMessage(cloud);

      if (syncPackages(meas)) {
        return true;
      }
    } else if (bag_msg->topic_name == bag_reader_->imu_topic) {
      auto imu_ptr = std::make_shared<sensor_msgs::msg::Imu>();
      bag_reader_->imu_ser.deserialize_message(&ser_msg, imu_ptr.get());
      pushImuMessage(imu_ptr);

      if (syncPackages(meas)) {
        return true;
      }
    }
  }

  // Bag exhausted — drain remaining buffered data
  eof_ = true;
  return syncPackages(meas);
}

// ------------------------------------------------------------------
// getSyncedData
// ------------------------------------------------------------------
bool SparkLioSdk::getSyncedData(MeasureGroup &meas) {
  if (!bag_reader_ || !bag_reader_->opened) return false;

  // First try with what's already in the buffers
  if (syncPackages(meas)) {
    return true;
  }

  // If buffers insufficient and bag not exhausted, read more
  if (!eof_) {
    return fillBuffersUntilSynced(meas);
  }

  return false;
}

}  // namespace spark_fast_lio