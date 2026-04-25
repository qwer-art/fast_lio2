/**
 * @file main.cpp
 * @brief SPARK-Fast-LIO SDK demo: read a rosbag2 and iterate synchronized LiDAR+IMU data.
 *
 * Usage:
 *   ros2 run spark_fast_lio spark_lio_sdk_demo <bag_path> [lidar_topic] [imu_topic]
 *
 * Example:
 *   ros2 run spark_fast_lio spark_lio_sdk_demo /data/velodyne_bag /velodyne_points /imu/data
 */

#include <iostream>
#include <string>

#include "spark_lio_sdk.h"

int main(int argc, char **argv) {
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <bag_path> [lidar_topic] [imu_topic]" << std::endl;
    std::cerr << "  lidar_topic default: /velodyne_points" << std::endl;
    std::cerr << "  imu_topic   default: /imu/data" << std::endl;
    return 1;
  }

  const std::string bag_path    = argv[1];
  const std::string lidar_topic = (argc > 2) ? argv[2] : "/velodyne_points";
  const std::string imu_topic   = (argc > 3) ? argv[3] : "/imu/data";

  // Configure the SDK
  spark_fast_lio::SparkLioSdk::Config config;
  config.lidar_type       = VELO16;  // Adjust for your LiDAR: AVIA, VELO16, OUST64, KMOUST64
  config.scan_line        = 16;
  config.scan_rate        = 10;
  config.time_unit        = US;
  config.blind            = 0.01;
  config.point_filter_num = 1;
  config.verbose          = true;

  // Open the bag
  spark_fast_lio::SparkLioSdk sdk;
  if (!sdk.open(bag_path, lidar_topic, imu_topic, config)) {
    std::cerr << "Failed to open bag: " << bag_path << std::endl;
    return 1;
  }

  // Iterate synchronized data
  MeasureGroup meas;
  int frame_count = 0;
  while (sdk.getSyncedData(meas)) {
    frame_count++;
    std::cout << "[Frame " << frame_count << "] "
              << "lidar_pts=" << meas.lidar->size()
              << "  imu_count=" << meas.imu.size()
              << "  t=[" << std::fixed << std::setprecision(6)
              << meas.lidar_beg_time << ", " << meas.lidar_end_time << "]"
              << std::endl;

    // --- Your algorithm goes here ---
    // meas.lidar        : PointCloudXYZI::Ptr  (preprocessed point cloud)
    // meas.lidar_beg_time : double (scan start time)
    // meas.lidar_end_time : double (scan end time)
    // meas.imu          : std::vector<sensor_msgs::msg::Imu::ConstSharedPtr>
  }

  std::cout << "Total synchronized frames: " << frame_count << std::endl;
  sdk.close();
  return 0;
}
