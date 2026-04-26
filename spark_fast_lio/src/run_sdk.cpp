/**
 * @file run_sdk.cpp
 * @brief SPARK-Fast-LIO SDK runner: read a rosbag2, run complete Fast-LIO2 algorithm,
 *        and save prediction/update states and covariances to disk.
 *
 * Usage:
 *   ros2 run spark_fast_lio spark_lio_run_sdk <bag_path> <asset_path>
 *
 * Example:
 *   ros2 run spark_fast_lio spark_lio_run_sdk /data/velodyne_bag /output/path
 */

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>

#include "spark_lio_sdk.h"
#include "fast_lio2_core.hpp"
#include "common/so3_math.h"

namespace fs = std::filesystem;

// ---------- Convert lidar_end_time (double seconds) to frame_time string ----------
static std::string toFrameTime(double end_time) {
  const int64_t HUNDRED_MS_PS = 100000000000LL;

  int64_t seconds = static_cast<int64_t>(end_time);
  double frac = end_time - static_cast<double>(seconds);

  int64_t picoseconds = static_cast<int64_t>(frac * 1e12 + 0.5);

  if (picoseconds >= 1000000000000LL) {
    picoseconds -= 1000000000000LL;
    seconds += 1;
  }

  int64_t rounded_ps_count = (picoseconds + HUNDRED_MS_PS / 2) / HUNDRED_MS_PS;
  picoseconds = rounded_ps_count * HUNDRED_MS_PS;

  if (picoseconds >= 1000000000000LL) {
    picoseconds -= 1000000000000LL;
    seconds += 1;
  }

  std::ostringstream oss;
  oss << std::setfill('0') << std::setw(12) << seconds << "_"
      << std::setfill('0') << std::setw(12) << picoseconds;
  return oss.str();
}

// ---------- Save state to JSON ----------
static bool saveStateJson(const std::string &filepath,
                          const std::string &frame_time,
                          const state_ikfom &state) {
  std::ofstream ofs(filepath);
  if (!ofs.is_open()) {
    std::cerr << "Failed to open: " << filepath << std::endl;
    return false;
  }

  Eigen::Vector3d euler = SO3ToEuler(state.rot);

  ofs << "{\n"
      << "  \"frame_time\": \"" << frame_time << "\",\n"
      << std::fixed << std::setprecision(9)
      << "  \"position\": [" << state.pos(0) << ", " << state.pos(1) << ", "
      << state.pos(2) << "],\n"
      << "  \"orientation_quat\": [" << state.rot.w() << ", " << state.rot.x()
      << ", " << state.rot.y() << ", " << state.rot.z() << "],\n"
      << "  \"orientation_euler_deg\": [" << euler(0) << ", " << euler(1) << ", "
      << euler(2) << "],\n"
      << "  \"velocity\": [" << state.vel(0) << ", " << state.vel(1) << ", "
      << state.vel(2) << "],\n"
      << "  \"bias_gyro\": [" << state.bg(0) << ", " << state.bg(1) << ", "
      << state.bg(2) << "],\n"
      << "  \"bias_acc\": [" << state.ba(0) << ", " << state.ba(1) << ", "
      << state.ba(2) << "],\n"
      << "  \"gravity\": [" << state.grav[0] << ", " << state.grav[1] << ", "
      << state.grav[2] << "],\n"
      << "  \"offset_R_L_I\": [" << state.offset_R_L_I.w() << ", "
      << state.offset_R_L_I.x() << ", " << state.offset_R_L_I.y() << ", "
      << state.offset_R_L_I.z() << "],\n"
      << "  \"offset_T_L_I\": [" << state.offset_T_L_I(0) << ", "
      << state.offset_T_L_I(1) << ", " << state.offset_T_L_I(2) << "]\n"
      << "}\n";
  ofs.close();
  return true;
}

// ---------- Save covariance to CSV ----------
static bool saveCovarianceCsv(const std::string &filepath,
                              const Eigen::Matrix<double, 23, 23> &cov) {
  std::ofstream ofs(filepath);
  if (!ofs.is_open()) {
    std::cerr << "Failed to open: " << filepath << std::endl;
    return false;
  }

  ofs << std::scientific << std::setprecision(6);
  for (int i = 0; i < cov.rows(); ++i) {
    for (int j = 0; j < cov.cols(); ++j) {
      ofs << cov(i, j);
      if (j < cov.cols() - 1) ofs << ",";
    }
    ofs << "\n";
  }
  ofs.close();
  return true;
}

// ========================================================================

int main(int argc, char **argv) {
  if (argc < 3) {
    std::cerr << "Usage: " << argv[0] << " <bag_path> <asset_path>" << std::endl;
    std::cerr << "  bag_path      : rosbag2 directory path" << std::endl;
    std::cerr << "  asset_path    : output directory for states and covariances"
              << std::endl;
    std::cerr << "  lidar_topic   : /hathor/lidar_points (hardcoded)" << std::endl;
    std::cerr << "  imu_topic     : /hathor/forward/imu (hardcoded)" << std::endl;
    return 1;
  }

  const std::string bag_path = argv[1];
  const std::string asset_path = argv[2];
  const std::string lidar_topic = "/hathor/lidar_points";
  const std::string imu_topic = "/hathor/forward/imu";

  // Configure the SDK for data synchronization
  spark_fast_lio::SparkLioSdk::Config config;
  config.lidar_type = VELO16;
  config.scan_line = 16;
  config.scan_rate = 10;
  config.time_unit = US;
  config.blind = 0.01;
  config.point_filter_num = 1;
  config.verbose = true;

  // Open the bag
  spark_fast_lio::SparkLioSdk sdk;
  if (!sdk.open(bag_path, lidar_topic, imu_topic, config)) {
    std::cerr << "Failed to open bag: " << bag_path << std::endl;
    return 1;
  }

  // Create output directories
  fs::create_directories(asset_path);
  fs::create_directories(fs::path(asset_path) / "pred_state");
  fs::create_directories(fs::path(asset_path) / "pred_cov");
  fs::create_directories(fs::path(asset_path) / "update_state");
  fs::create_directories(fs::path(asset_path) / "update_cov");

  // Initialize Fast-LIO2 algorithm core
  spark_fast_lio::FastLIO2Core::Config core_config;
  core_config.point_filter_num = 4;
  core_config.filter_size_map_min = 0.5;
  core_config.cube_len = 200.0;
  core_config.det_range = 300.0;
  core_config.max_iterations = 4;
  core_config.extrinsic_est_en = false;
  core_config.verbose = true;

  // Use heap allocation to avoid stack overflow (FastLIO2Core is ~80MB)
  auto core = std::make_unique<spark_fast_lio::FastLIO2Core>(core_config);

  // Iterate synchronized data and run Fast-LIO2 algorithm
  MeasureGroup meas;
  int frame_count = 0;

  while (sdk.getSyncedData(meas)) {
    frame_count++;

    std::string frame_time = toFrameTime(meas.lidar_end_time);

    std::cout << "[Frame " << frame_count << "] "
              << "frame_time=" << frame_time << "  "
              << "lidar_pts=" << meas.lidar->size() << "  imu_count="
              << meas.imu.size() << "  t=[" << std::fixed
              << std::setprecision(6) << meas.lidar_beg_time << ", "
              << meas.lidar_end_time << "]" << std::endl;

    // Process the measure group using FastLIO2Core
    core->processLidarAndImu(meas);

    // Save prediction state and covariance
    saveStateJson(
        (fs::path(asset_path) / "pred_state" / (frame_time + ".json")).string(),
        frame_time, core->getPredictionState());
    saveCovarianceCsv(
        (fs::path(asset_path) / "pred_cov" / (frame_time + ".csv")).string(),
        core->getPredictionCovariance());

    // Save update state and covariance
    saveStateJson(
        (fs::path(asset_path) / "update_state" / (frame_time + ".json")).string(),
        frame_time, core->getUpdateState());
    saveCovarianceCsv(
        (fs::path(asset_path) / "update_cov" / (frame_time + ".csv")).string(),
        core->getUpdateCovariance());
  }

  std::cout << "Total processed frames: " << frame_count << std::endl;
  std::cout << "States and covariances saved to: " << asset_path << std::endl;
  sdk.close();
  return 0;
}
