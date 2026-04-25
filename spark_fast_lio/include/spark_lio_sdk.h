#pragma once

#include <deque>
#include <memory>
#include <mutex>
#include <string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "common/common_lib.h"
#include "preprocess.h"

namespace spark_fast_lio {

class SparkLioSdk {
 public:
  struct Config {
    int lidar_type   = AVIA;   // 1=AVIA, 2=VELO16, 3=OUST64, 4=KMOUST64
    int scan_line    = 16;
    int scan_rate    = 10;
    int time_unit    = US;     // 0=SEC, 1=MS, 2=US, 3=NS
    double blind     = 0.01;
    int point_filter_num = 1;
    bool verbose     = false;
  };

  SparkLioSdk();
  ~SparkLioSdk();

  /// Open a rosbag2 directory for reading.
  /// @param bag_path  Directory containing metadata.yaml
  /// @param lidar_topic  Topic name for PointCloud2
  /// @param imu_topic    Topic name for Imu
  /// @param config       Preprocessor configuration
  bool open(const std::string &bag_path,
            const std::string &lidar_topic,
            const std::string &imu_topic,
            const Config &config);

  /// Get one synchronized MeasureGroup.
  /// Returns true if data was available, false if bag is exhausted.
  bool getSyncedData(MeasureGroup &meas);

  /// Close the bag reader and release resources.
  void close();

  /// Check if the bag has been fully read.
  bool isEof() const;

 private:
  // Read messages from bag and fill buffers until syncPackages succeeds or EOF.
  bool fillBuffersUntilSynced(MeasureGroup &meas);

  // Core sync logic (extracted from SPARKFastLIO2::syncPackages)
  bool syncPackages(MeasureGroup &meas);

  // Process a PointCloud2 message through the preprocessor and push to lidar buffer
  void pushLidarMessage(const sensor_msgs::msg::PointCloud2 &cloud_msg);

  // Push an IMU message to the imu buffer
  void pushImuMessage(const sensor_msgs::msg::Imu::ConstSharedPtr &imu_msg);

  // --- Bag reader (opaque to avoid exposing rosbag2 headers) ---
  struct BagReader;
  std::unique_ptr<BagReader> bag_reader_;

  // --- Buffers ---
  std::mutex buffer_mutex_;
  std::deque<PointCloudXYZI::Ptr> lidar_buffer_;
  std::deque<double> time_buffer_;
  std::deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu_buffer_;

  // --- Sync state ---
  bool lidar_pushed_          = false;
  double lidar_end_time_      = 0.0;
  double lidar_mean_scantime_ = 0.0;
  int scan_num_               = 0;
  double last_imu_time_       = 0.0;
  bool verbose_               = false;
  bool eof_                   = false;

  // --- Preprocessor ---
  std::shared_ptr<Preprocess> preprocessor_;
};

}  // namespace spark_fast_lio
