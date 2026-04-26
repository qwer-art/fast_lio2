/**
 * @file run_sdk.cpp
 * @brief SPARK-Fast-LIO SDK runner: read a rosbag2, run complete Fast-LIO2 algorithm,
 *        and save prediction/update states and covariances to disk.
 *
 * Usage:
 *   ros2 run spark_fast_lio spark_lio_run_sdk <bag_path> <asset_path> <param_path>
 *
 * Example:
 *   ros2 run spark_fast_lio spark_lio_run_sdk /data/velodyne_bag /output/path /config/kimera_multi/hathor.yaml
 */

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>

#include <yaml-cpp/yaml.h>

#include "spark_lio_sdk.h"
#include "fast_lio2_core.hpp"
#include "common/so3_math.h"

namespace fs = std::filesystem;

// ---------- Config structure ----------
struct RunConfig {
  // Preprocess parameters
  int lidar_type = 2;       // VELO16
  int scan_line = 16;
  int scan_rate = 10;
  int time_unit = 2;        // US
  double blind = 0.01;
  double blind_for_human_pilot = 1.5;
  int point_filter_num_preprocess = 1;

  // Mapping parameters
  double acc_cov = 0.1;
  double gyr_cov = 0.1;
  double b_acc_cov = 0.0001;
  double b_gyr_cov = 0.0001;
  double det_range = 300.0;
  bool extrinsic_est_en = false;
  std::vector<double> extrinsic_T;
  std::vector<double> extrinsic_R;

  // Other parameters
  double filter_size_map = 0.5;
  int point_filter_num = 4;
  int max_iteration = 4;
  double cube_side_length = 200.0;
  bool verbose = false;
};

// ---------- Load config from YAML ----------
bool loadConfig(const std::string &param_path, RunConfig &config) {
  try {
    YAML::Node yaml = YAML::LoadFile(param_path);

    // Navigate to ros__parameters
    YAML::Node params;
    if (yaml["/**"] && yaml["/**"]["ros__parameters"]) {
      params = yaml["/**"]["ros__parameters"];
    } else if (yaml["ros__parameters"]) {
      params = yaml["ros__parameters"];
    } else {
      params = yaml;
    }

    // Preprocess parameters
    if (params["preprocess"]) {
      auto pre = params["preprocess"];
      if (pre["lidar_type"]) config.lidar_type = pre["lidar_type"].as<int>();
      if (pre["scan_line"]) config.scan_line = pre["scan_line"].as<int>();
      if (pre["scan_rate"]) config.scan_rate = pre["scan_rate"].as<int>();
      if (pre["timestamp_unit"]) config.time_unit = pre["timestamp_unit"].as<int>();
      if (pre["blind"]) config.blind = pre["blind"].as<double>();
      if (pre["blind_for_human_pilot"]) config.blind_for_human_pilot = pre["blind_for_human_pilot"].as<double>();
    }

    // Mapping parameters
    if (params["mapping"]) {
      auto map = params["mapping"];
      if (map["acc_cov"]) config.acc_cov = map["acc_cov"].as<double>();
      if (map["gyr_cov"]) config.gyr_cov = map["gyr_cov"].as<double>();
      if (map["b_acc_cov"]) config.b_acc_cov = map["b_acc_cov"].as<double>();
      if (map["b_gyr_cov"]) config.b_gyr_cov = map["b_gyr_cov"].as<double>();
      if (map["det_range"]) config.det_range = map["det_range"].as<double>();
      if (map["extrinsic_est_en"]) config.extrinsic_est_en = map["extrinsic_est_en"].as<bool>();
      if (map["extrinsic_T"]) config.extrinsic_T = map["extrinsic_T"].as<std::vector<double>>();
      if (map["extrinsic_R"]) config.extrinsic_R = map["extrinsic_R"].as<std::vector<double>>();
    }

    // Other parameters
    if (params["filter_size_map"]) config.filter_size_map = params["filter_size_map"].as<double>();
    if (params["point_filter_num_for_preprocessing"])
      config.point_filter_num_preprocess = params["point_filter_num_for_preprocessing"].as<int>();
    if (params["point_filter_num"]) config.point_filter_num = params["point_filter_num"].as<int>();
    if (params["max_iteration"]) config.max_iteration = params["max_iteration"].as<int>();
    if (params["cube_side_length"]) config.cube_side_length = params["cube_side_length"].as<double>();
    if (params["verbose"]) config.verbose = params["verbose"].as<bool>();

    return true;
  } catch (const std::exception &e) {
    std::cerr << "Failed to load config: " << e.what() << std::endl;
    return false;
  }
}

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

// ---------- Save state to CSV ----------
static bool saveStateCsv(const std::string &filepath,
                         const std::string &frame_time,
                         const state_ikfom &state) {
  std::ofstream ofs(filepath);
  if (!ofs.is_open()) {
    std::cerr << "Failed to open: " << filepath << std::endl;
    return false;
  }

  Eigen::Vector3d euler = SO3ToEuler(state.rot);

  ofs << std::fixed << std::setprecision(9)
      << frame_time << ","
      << state.pos(0) << "," << state.pos(1) << "," << state.pos(2) << ","
      << state.rot.w() << "," << state.rot.x() << "," << state.rot.y() << "," << state.rot.z() << ","
      << euler(0) << "," << euler(1) << "," << euler(2) << ","
      << state.vel(0) << "," << state.vel(1) << "," << state.vel(2) << ","
      << state.bg(0) << "," << state.bg(1) << "," << state.bg(2) << ","
      << state.ba(0) << "," << state.ba(1) << "," << state.ba(2) << ","
      << state.grav[0] << "," << state.grav[1] << "," << state.grav[2] << ","
      << state.offset_R_L_I.w() << "," << state.offset_R_L_I.x() << ","
      << state.offset_R_L_I.y() << "," << state.offset_R_L_I.z() << ","
      << state.offset_T_L_I(0) << "," << state.offset_T_L_I(1) << "," << state.offset_T_L_I(2)
      << "\n";
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
  if (argc < 4) {
    std::cerr << "Usage: " << argv[0] << " <bag_path> <asset_path> <param_path>" << std::endl;
    std::cerr << "  bag_path   : rosbag2 directory path" << std::endl;
    std::cerr << "  asset_path : output directory for states and covariances" << std::endl;
    std::cerr << "  param_path : YAML config file path (e.g., config/kimera_multi/hathor.yaml)" << std::endl;
    return 1;
  }

  const std::string bag_path = argv[1];
  const std::string asset_path = argv[2];
  const std::string param_path = argv[3];
  const std::string lidar_topic = "/hathor/lidar_points";
  const std::string imu_topic = "/hathor/forward/imu";

  // Load config from YAML
  RunConfig config;
  if (!loadConfig(param_path, config)) {
    std::cerr << "Error: Failed to load config from " << param_path << std::endl;
    return 1;
  }

  std::cout << "========================================" << std::endl;
  std::cout << "Loaded config from: " << param_path << std::endl;
  std::cout << "========================================" << std::endl;
  std::cout << "Preprocess:" << std::endl;
  std::cout << "  lidar_type: " << config.lidar_type << std::endl;
  std::cout << "  scan_line: " << config.scan_line << std::endl;
  std::cout << "  scan_rate: " << config.scan_rate << std::endl;
  std::cout << "  time_unit: " << config.time_unit << std::endl;
  std::cout << "  blind: " << config.blind << std::endl;
  std::cout << "  blind_for_human_pilot: " << config.blind_for_human_pilot << std::endl;
  std::cout << "  point_filter_num_preprocess: " << config.point_filter_num_preprocess << std::endl;
  std::cout << "Mapping:" << std::endl;
  std::cout << "  acc_cov: " << config.acc_cov << std::endl;
  std::cout << "  gyr_cov: " << config.gyr_cov << std::endl;
  std::cout << "  b_acc_cov: " << config.b_acc_cov << std::endl;
  std::cout << "  b_gyr_cov: " << config.b_gyr_cov << std::endl;
  std::cout << "  det_range: " << config.det_range << std::endl;
  std::cout << "  filter_size_map: " << config.filter_size_map << std::endl;
  std::cout << "  point_filter_num: " << config.point_filter_num << std::endl;
  std::cout << "  max_iteration: " << config.max_iteration << std::endl;
  std::cout << "  cube_side_length: " << config.cube_side_length << std::endl;
  std::cout << "  extrinsic_est_en: " << config.extrinsic_est_en << std::endl;
  if (config.extrinsic_T.size() == 3) {
    std::cout << "  extrinsic_T: [" << config.extrinsic_T[0] << ", " << config.extrinsic_T[1] << ", " << config.extrinsic_T[2] << "]" << std::endl;
  }
  std::cout << "========================================" << std::endl;

  // Configure the SDK for data synchronization
  spark_fast_lio::SparkLioSdk::Config sdk_config;
  sdk_config.lidar_type = static_cast<LID_TYPE>(config.lidar_type);
  sdk_config.scan_line = config.scan_line;
  sdk_config.scan_rate = config.scan_rate;
  sdk_config.time_unit = static_cast<TIME_UNIT>(config.time_unit);
  sdk_config.blind = config.blind;
  sdk_config.blind_for_human_pilot = config.blind_for_human_pilot;
  sdk_config.point_filter_num = config.point_filter_num_preprocess;
  sdk_config.verbose = config.verbose;

  // Open the bag
  spark_fast_lio::SparkLioSdk sdk;
  if (!sdk.open(bag_path, lidar_topic, imu_topic, sdk_config)) {
    std::cerr << "Failed to open bag: " << bag_path << std::endl;
    return 1;
  }

  // Create output directories
  fs::create_directories(asset_path);
  fs::create_directories(fs::path(asset_path) / "pred_state");
  fs::create_directories(fs::path(asset_path) / "pred_cov");
  fs::create_directories(fs::path(asset_path) / "update_state");
  fs::create_directories(fs::path(asset_path) / "update_cov");

  // Write CSV headers for state files
  std::ofstream pred_header(fs::path(asset_path) / "pred_state" / "header.txt");
  pred_header << "frame_time,pos_x,pos_y,pos_z,quat_w,quat_x,quat_y,quat_z,"
              << "euler_roll,euler_pitch,euler_yaw,"
              << "vel_x,vel_y,vel_z,"
              << "bg_x,bg_y,bg_z,"
              << "ba_x,ba_y,ba_z,"
              << "grav_x,grav_y,grav_z,"
              << "Ril_w,Ril_x,Ril_y,Ril_z,"
              << "Til_x,Til_y,Til_z\n";
  pred_header.close();

  std::ofstream update_header(fs::path(asset_path) / "update_state" / "header.txt");
  update_header << "frame_time,pos_x,pos_y,pos_z,quat_w,quat_x,quat_y,quat_z,"
                << "euler_roll,euler_pitch,euler_yaw,"
                << "vel_x,vel_y,vel_z,"
                << "bg_x,bg_y,bg_z,"
                << "ba_x,ba_y,ba_z,"
                << "grav_x,grav_y,grav_z,"
                << "Ril_w,Ril_x,Ril_y,Ril_z,"
                << "Til_x,Til_y,Til_z\n";
  update_header.close();

  // Initialize Fast-LIO2 algorithm core
  spark_fast_lio::FastLIO2Core::Config core_config;
  core_config.point_filter_num = config.point_filter_num;
  core_config.filter_size_map_min = config.filter_size_map;
  core_config.cube_len = config.cube_side_length;
  core_config.det_range = config.det_range;
  core_config.max_iterations = config.max_iteration;
  core_config.extrinsic_est_en = config.extrinsic_est_en;
  core_config.verbose = config.verbose;

  // IMU covariance parameters
  core_config.gyr_cov = Eigen::Vector3d(config.gyr_cov, config.gyr_cov, config.gyr_cov);
  core_config.acc_cov = Eigen::Vector3d(config.acc_cov, config.acc_cov, config.acc_cov);
  core_config.gyr_bias_cov = Eigen::Vector3d(config.b_gyr_cov, config.b_gyr_cov, config.b_gyr_cov);
  core_config.acc_bias_cov = Eigen::Vector3d(config.b_acc_cov, config.b_acc_cov, config.b_acc_cov);

  // LiDAR-IMU extrinsic parameters
  if (config.extrinsic_T.size() == 3 && config.extrinsic_R.size() == 9) {
    core_config.lidar_T_wrt_imu = Eigen::Vector3d(config.extrinsic_T[0], config.extrinsic_T[1], config.extrinsic_T[2]);
    core_config.lidar_R_wrt_imu << config.extrinsic_R[0], config.extrinsic_R[1], config.extrinsic_R[2],
                                     config.extrinsic_R[3], config.extrinsic_R[4], config.extrinsic_R[5],
                                     config.extrinsic_R[6], config.extrinsic_R[7], config.extrinsic_R[8];
  }

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
    saveStateCsv(
        (fs::path(asset_path) / "pred_state" / (frame_time + ".csv")).string(),
        frame_time, core->getPredictionState());
    saveCovarianceCsv(
        (fs::path(asset_path) / "pred_cov" / (frame_time + ".csv")).string(),
        core->getPredictionCovariance());

    // Save update state and covariance
    saveStateCsv(
        (fs::path(asset_path) / "update_state" / (frame_time + ".csv")).string(),
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
