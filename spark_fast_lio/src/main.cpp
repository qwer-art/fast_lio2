/**
 * @file main.cpp
 * @brief SPARK-Fast-LIO SDK demo: read a rosbag2, iterate synchronized LiDAR+IMU data,
 *        and serialize to disk (frame_info JSON, lidar PCD, imu CSV).
 *
 * Usage:
 *   ros2 run spark_fast_lio spark_lio_sdk_demo <bag_path> [lidar_topic] [imu_topic]
 *
 * Example:
 *   ros2 run spark_fast_lio spark_lio_sdk_demo /data/velodyne_bag /velodyne_points /imu/data
 */

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

#include "spark_lio_sdk.h"

namespace fs = std::filesystem;

// ---------- Output root directory ----------
static const std::string asset_data{
    "/home/jerett/OpenProject/LidarSlam/spark-fast-lio/spark_fast_lio/data/10_14_hathor/asset_data"};

// ---------- Convert end_time (double seconds) to frame_time string ----------
// Use 100ms as base unit, round to nearest 100ms boundary
// Format: seconds_picoseconds (both 12 digits, e.g., 001666028921_450000000000)
static std::string toFrameTime(double end_time) {
  // 100ms = 0.1s = 100000000000 ps
  const int64_t HUNDRED_MS_PS = 100000000000LL;

  // Split into integer seconds and fractional seconds first to avoid overflow
  int64_t seconds = static_cast<int64_t>(end_time);
  double frac = end_time - static_cast<double>(seconds);

  // Convert fractional part to picoseconds (frac is in [0, 1), so frac*1e12 fits in int64)
  int64_t picoseconds = static_cast<int64_t>(frac * 1e12 + 0.5);

  // Handle rounding overflow (if picoseconds >= 1 second)
  if (picoseconds >= 1000000000000LL) {
    picoseconds -= 1000000000000LL;
    seconds += 1;
  }

  // Round picoseconds to nearest 100ms (100000000000 ps)
  int64_t rounded_ps_count = (picoseconds + HUNDRED_MS_PS / 2) / HUNDRED_MS_PS;
  picoseconds = rounded_ps_count * HUNDRED_MS_PS;

  // Handle rounding overflow again (if picoseconds >= 1 second)
  if (picoseconds >= 1000000000000LL) {
    picoseconds -= 1000000000000LL;
    seconds += 1;
  }

  // Format: seconds_picoseconds (both 12 digits with padding)
  std::ostringstream oss;
  oss << std::setfill('0') << std::setw(12) << seconds << "_"
      << std::setfill('0') << std::setw(12) << picoseconds;
  return oss.str();
}

// ---------- Save frame_info JSON ----------
static bool saveFrameInfo(const std::string &frame_time,
                          double begin_time, double end_time) {
  fs::path dir = fs::path(asset_data) / "frame_info";
  fs::create_directories(dir);

  fs::path filepath = dir / (frame_time + ".json");
  std::ofstream ofs(filepath);
  if (!ofs.is_open()) {
    std::cerr << "Failed to open: " << filepath << std::endl;
    return false;
  }

  ofs << "{\n"
      << "  \"frame_time\": \"" << frame_time << "\",\n"
      << std::fixed << std::setprecision(9)
      << "  \"begin_time\": " << begin_time << ",\n"
      << "  \"end_time\": " << end_time << "\n"
      << "}\n";
  ofs.close();
  return true;
}

// ---------- Save lidar PLY ----------
// Write only x, y, z, intensity to avoid CloudCompare warnings about
// empty normal/curvature fields and camera/face elements that PCL adds.
static bool saveLidarPly(const std::string &frame_time,
                         const PointCloudXYZI::Ptr &cloud) {
  fs::path dir = fs::path(asset_data) / "lidar";
  fs::create_directories(dir);

  fs::path filepath = dir / (frame_time + ".ply");
  std::ofstream ofs(filepath);
  if (!ofs.is_open()) {
    std::cerr << "Failed to open: " << filepath << std::endl;
    return false;
  }

  // PLY header: only vertex with x, y, z, intensity
  ofs << "ply\n"
      << "format ascii 1.0\n"
      << "element vertex " << cloud->size() << "\n"
      << "property float x\n"
      << "property float y\n"
      << "property float z\n"
      << "property float intensity\n"
      << "end_header\n";

  ofs << std::fixed << std::setprecision(6);
  for (const auto &pt : cloud->points) {
    ofs << pt.x << " " << pt.y << " " << pt.z << " " << pt.intensity << "\n";
  }
  ofs.close();
  return true;
}

// ---------- Save imu CSV ----------
static bool saveImuCsv(const std::string &frame_time,
                       const std::deque<std::shared_ptr<const sensor_msgs::msg::Imu>> &imu_data) {
  fs::path dir = fs::path(asset_data) / "imu";
  fs::create_directories(dir);

  fs::path filepath = dir / (frame_time + ".csv");
  std::ofstream ofs(filepath);
  if (!ofs.is_open()) {
    std::cerr << "Failed to open: " << filepath << std::endl;
    return false;
  }

  // Header
  ofs << "timestamp,ax,ay,az,gx,gy,gz,qx,qy,qz,qw\n";

  // Data rows
  ofs << std::fixed << std::setprecision(9);
  for (const auto &imu : imu_data) {
    double t = imu->header.stamp.sec + imu->header.stamp.nanosec * 1e-9;
    ofs << t << ","
        << imu->linear_acceleration.x << ","
        << imu->linear_acceleration.y << ","
        << imu->linear_acceleration.z << ","
        << imu->angular_velocity.x << ","
        << imu->angular_velocity.y << ","
        << imu->angular_velocity.z << ","
        << imu->orientation.x << ","
        << imu->orientation.y << ","
        << imu->orientation.z << ","
        << imu->orientation.w << "\n";
  }
  ofs.close();
  return true;
}

// ========================================================================

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
  config.scan_rate        = 10;  // 10Hz = 100ms per frame
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

  // Create output root directory
  fs::create_directories(asset_data);

  // Iterate synchronized data and serialize
  MeasureGroup meas;
  int frame_count = 0;
  while (sdk.getSyncedData(meas)) {
    frame_count++;

    // Convert end_time to frame_time string (nanoseconds integer)
    std::string frame_time = toFrameTime(meas.lidar_end_time);

    std::cout << "[Frame " << frame_count << "] "
              << "frame_time=" << frame_time << "  "
              << "lidar_pts=" << meas.lidar->size()
              << "  imu_count=" << meas.imu.size()
              << "  t=[" << std::fixed << std::setprecision(6)
              << meas.lidar_beg_time << ", " << meas.lidar_end_time << "]"
              << std::endl;

    // 1. Save frame_info JSON
    saveFrameInfo(frame_time, meas.lidar_beg_time, meas.lidar_end_time);

    // 2. Save lidar PLY
    saveLidarPly(frame_time, meas.lidar);

    // 3. Save imu CSV
    saveImuCsv(frame_time, meas.imu);
  }

  std::cout << "Total synchronized frames: " << frame_count << std::endl;
  std::cout << "Data saved to: " << asset_data << std::endl;
  sdk.close();
  return 0;
}
