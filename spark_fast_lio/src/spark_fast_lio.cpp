#include "spark_fast_lio.h"

#include <cmath>
#include <stdexcept>

#include <omp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp_components/register_node_macro.hpp>

namespace spark_fast_lio {

SPARKFastLIO2::SPARKFastLIO2(const rclcpp::NodeOptions &options)
    : Node("spark_fast_lio_node", options),
      clock_(get_clock()),
      last_lidar_timestamp_(now()),
      last_imu_timestamp_(now()) {
  preprocessor_ = std::make_shared<Preprocess>();

  g_base_           = Zero3d;
  mean_acc_stopped_ = Zero3d;
  R_gravity_aligned_ = Eye3d;
  lidar_T_wrt_base_ = Zero3d;
  lidar_R_wrt_base_ = Eye3d;

  path_en_           = declare_parameter<bool>("publish.path_en", true);
  scan_pub_en_       = declare_parameter<bool>("publish.scan_publish_en", false);
  dense_pub_en_      = declare_parameter<bool>("publish.dense_publish_en", false);
  scan_lidar_pub_en_ = declare_parameter<bool>("publish.scan_lidarframe_pub_en", false);
  scan_body_pub_en_  = declare_parameter<bool>("publish.scan_bodyframe_pub_en", false);
  scan_base_pub_en_  = declare_parameter<bool>("publish.scan_baseframe_pub_en", false);

  map_frame_     = declare_parameter<std::string>("common.map_frame", "odom");
  lidar_frame_   = declare_parameter<std::string>("common.lidar_frame", "lidar");
  base_frame_    = declare_parameter<std::string>("common.base_frame", "");
  imu_frame_     = declare_parameter<std::string>("common.imu_frame", "imu");
  viz_frame_     = declare_parameter<std::string>("common.visualization_frame", "imu");
  time_sync_en_  = declare_parameter<bool>("common.time_sync_en", false);

  double filter_size_map_min = declare_parameter<double>("filter_size_map", 0.5);
  double cube_len            = declare_parameter<double>("cube_side_length", 200.0);
  double det_range           = declare_parameter<double>("mapping.det_range", 300.0);
  double gyr_cov             = declare_parameter<double>("mapping.gyr_cov", 0.1);
  double acc_cov             = declare_parameter<double>("mapping.acc_cov", 0.1);
  double b_gyr_cov           = declare_parameter<double>("mapping.b_gyr_cov", 0.0001);
  double b_acc_cov           = declare_parameter<double>("mapping.b_acc_cov", 0.0001);
  bool extrinsic_est_en      = declare_parameter<bool>("mapping.extrinsic_est_en", false);

  enable_gravity_alignment_ =
      declare_parameter<bool>("gravity_alignment.enable_gravity_alignment", true);
  acc_diff_thr_          = declare_parameter<double>("gravity_alignment.acc_diff_thr", 0.2);
  num_moving_frames_thr_ = declare_parameter<int>("gravity_alignment.num_moving_frames_thr", 20);
  num_gravity_measurements_thr_ =
      declare_parameter<int>("gravity_alignment.num_gravity_measurements_thr", 20);

  verbose_     = declare_parameter<bool>("verbose", false);
  pcl_verbose_ = declare_parameter<bool>("pcl_verbose", true);
  if (!pcl_verbose_) {
    pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
  }

  runtime_pos_log_      = declare_parameter<bool>("runtime_pos_log_enable", false);
  extrinsics_timeout_s_ = declare_parameter<double>("extrinsics_timeout_s", 10.0);
  pcd_save_en_          = declare_parameter<bool>("pcd_save.pcd_save_en", false);
  pcd_save_interval_    = declare_parameter<int>("pcd_save.interval", -1);

  int point_filter_num = declare_parameter<int>("point_filter_num", 4);

  // extrinT_ and extrinR_ are sized 3 and 9 respectively
  extrinT_ = declare_parameter<std::vector<double>>("mapping.extrinsic_T", extrinT_);
  extrinR_ = declare_parameter<std::vector<double>>("mapping.extrinsic_R", extrinR_);

  auto g_vec = declare_parameter<std::vector<double>>("gravity_alignment.g_base", {0.0, 0.0, -1.0});
  g_base_ << g_vec[0], g_vec[1], g_vec[2];

  // ========== Construct FastLIO2Core ==========
  FastLIO2Core::Config core_config;
  core_config.point_filter_num = point_filter_num;
  core_config.filter_size_map_min = filter_size_map_min;
  core_config.cube_len = cube_len;
  core_config.det_range = det_range;
  core_config.max_iterations = declare_parameter<int>("max_iteration", 4);
  core_config.extrinsic_est_en = extrinsic_est_en;
  core_config.verbose = verbose_;
  core_config.gyr_cov = V3D(gyr_cov, gyr_cov, gyr_cov);
  core_config.acc_cov = V3D(acc_cov, acc_cov, acc_cov);
  core_config.gyr_bias_cov = V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov);
  core_config.acc_bias_cov = V3D(b_acc_cov, b_acc_cov, b_acc_cov);

  if (extrinT_.size() == 3 && extrinR_.size() == 9) {
    core_config.lidar_T_wrt_imu = V3D(extrinT_[0], extrinT_[1], extrinT_[2]);
    core_config.lidar_R_wrt_imu << extrinR_[0], extrinR_[1], extrinR_[2],
        extrinR_[3], extrinR_[4], extrinR_[5], extrinR_[6], extrinR_[7], extrinR_[8];
  }

  core_ = FastLIO2Core(core_config);

  // ========== ROS subscriptions ==========
  rclcpp::QoS lidar_qos(rclcpp::KeepLast(10));
  lidar_qos.reliable();
  lidar_qos.durability_volatile();
  sub_lidar_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "lidar",
      lidar_qos,
      std::bind(&SPARKFastLIO2::standardLiDARCallback, this, std::placeholders::_1));

#if defined(LIVOX_ROS_DRIVER_FOUND) && LIVOX_ROS_DRIVER_FOUND
  sub_lidar_livox_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
      "lidar",
      lidar_qos,
      std::bind(&SPARKFastLIO2::livoxLidarCallback, this, std::placeholders::_1));
#endif
  auto imu_qos = rclcpp::SensorDataQoS();
  sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
      "imu", imu_qos, std::bind(&SPARKFastLIO2::imuCallback, this, std::placeholders::_1));

  // ========== ROS publishers ==========
  rclcpp::QoS qos((rclcpp::SystemDefaultsQoS().keep_last(1).durability_volatile()));
  pub_cloud_full_  = create_publisher<sensor_msgs::msg::PointCloud2>("cloud_registered", qos);
  pub_cloud_lidar_ = create_publisher<sensor_msgs::msg::PointCloud2>("cloud_registered_lidar", qos);
  pub_cloud_body_  = create_publisher<sensor_msgs::msg::PointCloud2>("cloud_registered_body", qos);
  pub_cloud_base_  = create_publisher<sensor_msgs::msg::PointCloud2>("cloud_registered_base", qos);

  pub_odom_                 = create_publisher<nav_msgs::msg::Odometry>("odometry", qos);
  pub_path_                 = create_publisher<nav_msgs::msg::Path>("path", qos);
  path_msg_.header.frame_id = map_frame_;

  // Debug publishers
  pub_debug_preint_pose_ = create_publisher<geometry_msgs::msg::PoseStamped>("debug/imu_preint_pose", qos);
  pub_debug_delta_pose_  = create_publisher<geometry_msgs::msg::Pose>("debug/delta_pose", qos);
  pub_debug_bias_gyro_   = create_publisher<geometry_msgs::msg::Vector3Stamped>("debug/imu_bias_gyro", qos);
  pub_debug_bias_acc_    = create_publisher<geometry_msgs::msg::Vector3Stamped>("debug/imu_bias_acc", qos);
  pub_debug_quality_     = create_publisher<std_msgs::msg::Float64MultiArray>("debug/match_quality", qos);
  pub_debug_velocity_    = create_publisher<geometry_msgs::msg::Vector3Stamped>("debug/velocity", qos);

  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  tf_buffer_      = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_    = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  preprocessor_        = std::make_shared<Preprocess>();
  preprocessor_->blind = declare_parameter<double>("preprocess.blind", 0.01);
  preprocessor_->blind_for_human_pilots =
      declare_parameter<double>("preprocess.blind_for_human_pilots", 1.5);
  preprocessor_->lidar_type =
      declare_parameter<int>("preprocess.lidar_type", static_cast<int>(AVIA));
  preprocessor_->N_SCANS = declare_parameter<int>("preprocess.scan_line", 16);
  preprocessor_->time_unit =
      declare_parameter<int>("preprocess.timestamp_unit", static_cast<int>(US));
  preprocessor_->SCAN_RATE        = declare_parameter<int>("preprocess.scan_rate", 10);
  preprocessor_->point_filter_num = declare_parameter<int>("point_filter_num_for_preprocessing", 1);

  cloud_to_be_saved_.reset(new PointCloudXYZI());

  if (!base_frame_.empty()) {
    if (!lookupBaseExtrinsics(lidar_T_wrt_base_, lidar_R_wrt_base_)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to lookup transform.");
      return;
    }
  }

  main_loop_timer_ =
      create_wall_timer(std::chrono::milliseconds(1), std::bind(&SPARKFastLIO2::main, this));

  RCLCPP_INFO(this->get_logger(), "SPARKFastLIO2 constructed (composition with FastLIO2Core)");
}

// Outputs rotation matrix that aligns a to b, i.e., R such that R * g_a = g_b
M3D SPARKFastLIO2::computeRelativeRotation(const Eigen::Vector3d &g_a, const Eigen::Vector3d &g_b) {
  Eigen::Vector3d g_a_norm = g_a.normalized();
  Eigen::Vector3d g_b_norm = g_b.normalized();

  Eigen::Vector3d axis = g_a_norm.cross(g_b_norm);
  double cos_theta     = g_a_norm.dot(g_b_norm);

  if (std::fabs(1.0 - cos_theta) < 1e-3) {
    return Eigen::Matrix3d::Identity();
  }

  // Degenerate condition a = -b
  if (std::fabs(1.0 + cos_theta) < 1e-3) {
    Eigen::Vector3d perturbed = g_a_norm + Eigen::Vector3d(1, 2, 3);
    axis                      = g_a_norm.cross(perturbed);

    if (axis.norm() < 1e-6) {
      perturbed = g_a_norm + Eigen::Vector3d(3, 2, 1);
      axis      = g_a_norm.cross(perturbed);
    }

    axis.normalize();
    return Eigen::AngleAxisd(M_PI, axis).toRotationMatrix();
  } else {
    axis.normalize();
    double theta = std::acos(cos_theta);

    Eigen::Quaterniond q(Eigen::AngleAxisd(theta, axis));

    return q.toRotationMatrix();
  }
}

bool SPARKFastLIO2::lookupBaseExtrinsics(V3D &lidar_T_wrt_base, M3D &lidar_R_wrt_base) {
  RCLCPP_INFO(this->get_logger(),
              "Looking up transform from %s -> %s",
              base_frame_.c_str(),
              lidar_frame_.c_str());

  const auto lookup_time = rclcpp::Time(0);
  bool has_transform     = false;
  std::string err_str;
  auto start_time          = this->now();
  rclcpp::Duration timeout = rclcpp::Duration::from_seconds(extrinsics_timeout_s_);
  rclcpp::Rate rate(10.0);

  while (rclcpp::ok()) {
    if (tf_buffer_->canTransform(
            base_frame_, lidar_frame_, lookup_time, tf2::durationFromSec(0.0), &err_str)) {
      RCLCPP_INFO_STREAM(this->get_logger(), "\033[1;32mExtrinsics detected.\033[1;0m");
      has_transform = true;
      break;
    }

    const auto time_since_start = now() - start_time;
    if (extrinsics_timeout_s_ > 0.0 && time_since_start > timeout) {
      RCLCPP_ERROR_STREAM(this->get_logger(),
                          "Timeout after "
                              << timeout.seconds() << " seconds waiting for transform from '"
                              << lidar_frame_ << "' to '" << base_frame_ << "': " << err_str);
      break;
    }

    RCLCPP_WARN_STREAM_SKIPFIRST_THROTTLE(get_logger(),
                                          *clock_,
                                          5000,
                                          "Waiting for transform from '" << lidar_frame_ << "' to '"
                                                                         << base_frame_
                                                                         << "': " << err_str);

    rate.sleep();
  }

  if (!has_transform) {
    return has_transform;
  }

  const auto &transform = tf_buffer_->lookupTransform(base_frame_, lidar_frame_, lookup_time);
  lidar_T_wrt_base(0)   = transform.transform.translation.x;
  lidar_T_wrt_base(1)   = transform.transform.translation.y;
  lidar_T_wrt_base(2)   = transform.transform.translation.z;

  Eigen::Quaterniond q(transform.transform.rotation.w,
                       transform.transform.rotation.x,
                       transform.transform.rotation.y,
                       transform.transform.rotation.z);

  lidar_R_wrt_base = q.toRotationMatrix();

  RCLCPP_INFO(this->get_logger(),
              "Translation: [%.3f, %.3f, %.3f]",
              lidar_T_wrt_base(0),
              lidar_T_wrt_base(1),
              lidar_T_wrt_base(2));

  RCLCPP_INFO(this->get_logger(),
              "Rotation (Quaternion): [%.3f, %.3f, %.3f, %.3f]",
              q.x(),
              q.y(),
              q.z(),
              q.w());

  return has_transform;
}

void SPARKFastLIO2::pclPointBodyToWorld(PointType const *const pi, PointType *const po) {
  const auto &latest_state = core_.getLatestState();
  V3D p_body(pi->x, pi->y, pi->z);
  V3D p_global(latest_state.rot *
                   (latest_state.offset_R_L_I * p_body + latest_state.offset_T_L_I) +
               latest_state.pos);

  po->x         = p_global(0);
  po->y         = p_global(1);
  po->z         = p_global(2);
  po->intensity = pi->intensity;
}

void SPARKFastLIO2::pclPointBodyLidarToIMU(PointType const *const pi, PointType *const po) {
  const auto &latest_state = core_.getLatestState();
  V3D p_body_lidar(pi->x, pi->y, pi->z);
  V3D p_body_imu(latest_state.offset_R_L_I * p_body_lidar + latest_state.offset_T_L_I);

  po->x         = p_body_imu(0);
  po->y         = p_body_imu(1);
  po->z         = p_body_imu(2);
  po->intensity = pi->intensity;
}

void SPARKFastLIO2::pclPointBodyLidarToBase(PointType const *const pi, PointType *const po) {
  V3D p_body_lidar(pi->x, pi->y, pi->z);
  V3D p_body_base(lidar_R_wrt_base_ * p_body_lidar + lidar_T_wrt_base_);

  po->x         = p_body_base(0);
  po->y         = p_body_base(1);
  po->z         = p_body_base(2);
  po->intensity = pi->intensity;
}

void SPARKFastLIO2::pclPointIMUToLiDAR(PointType const *const pi, PointType *const po) {
  const auto &latest_state = core_.getLatestState();
  V3D p_body_imu(pi->x, pi->y, pi->z);
  V3D p_body_lidar(latest_state.offset_R_L_I.inverse() *
                   (p_body_imu - latest_state.offset_T_L_I));

  po->x         = p_body_lidar(0);
  po->y         = p_body_lidar(1);
  po->z         = p_body_lidar(2);
  po->intensity = pi->intensity;
}

void SPARKFastLIO2::pclPointIMUToBase(PointType const *const pi, PointType *const po) {
  const auto &latest_state = core_.getLatestState();
  const auto offset_R_B_I = latest_state.offset_R_L_I * lidar_R_wrt_base_.inverse();
  const auto offset_T_B_I =
      -1 * offset_R_B_I * lidar_T_wrt_base_ + latest_state.offset_T_L_I;

  V3D p_body_imu(pi->x, pi->y, pi->z);
  V3D p_body_base(offset_R_B_I.inverse() * (p_body_imu - offset_T_B_I));

  po->x         = p_body_base(0);
  po->y         = p_body_base(1);
  po->z         = p_body_base(2);
  po->intensity = pi->intensity;
}

void SPARKFastLIO2::standardLiDARCallback(const sensor_msgs::msg::PointCloud2 &msg) {
  std::lock_guard<std::mutex> lk(buffer_mutex_);
  scan_count_++;
  rclcpp::Time msg_time = msg.header.stamp;

  if (msg_time < last_lidar_timestamp_) {
    RCLCPP_ERROR(get_logger(), "Lidar loopback detected, clearing buffers");
    lidar_buffer_.clear();
  }
  last_lidar_timestamp_ = msg_time;

  PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
  preprocessor_->process(msg, ptr);

  lidar_buffer_.push_back(ptr);
  time_buffer_.push_back(msg_time.seconds());
}

#if defined(LIVOX_ROS_DRIVER_FOUND) && LIVOX_ROS_DRIVER_FOUND
void SPARKFastLIO2::livoxLiDARCallback(
    const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr msg) {
  static bool timediff_set_flg = false;

  std::lock_guard<std::mutex> lk(buffer_mutex_);
  scan_count_++;
  rclcpp::Time msg_time = msg.header.stamp;

  if (msg_time < last_lidar_timestamp_) {
    RCLCPP_ERROR(get_logger(), "Livox loopback, clearing buffers");
    lidar_buffer_.clear();
  }
  last_lidar_timestamp_ = msg_time;

  const auto diff_s = std::abs((last_imu_timestamp_ - last_lidar_timestamp_).seconds());
  if (!time_sync_en_ && diff_s > 10.0 && !imu_buffer_.empty() && !lidar_buffer_.empty()) {
    RCLCPP_WARN_STREAM(this->get_logger(),
                       "IMU and LiDAR not Synced, IMU time: "
                           << last_imu_timestamp_.nanoseconds()
                           << ", lidar header time: " << last_lidar_timestamp_.nanoseconds());
  }

  if (time_sync_en_ && !timediff_set_flg && diff_s > 1.0 && !imu_buffer_.empty()) {
    timediff_set_flg        = true;
    timediff_lidar_wrt_imu_ = last_lidar_timestamp_.nanoseconds() + static_cast<int64_t>(1.0e8) -
                              last_imu_timestamp_.nanoseconds();
    RCLCPP_INFO_STREAM(
        this->get_logger(),
        "Self sync IMU and LiDAR, time diff is " << timediff_lidar_wrt_imu_ << "[ns]");
  }

  PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
  preprocessor_->process(msg, ptr);

  lidar_buffer_.push_back(ptr);
  time_buffer_.push_back(msg_time.seconds());
}
#endif

void SPARKFastLIO2::imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr msg) {
  ++publish_count_;

  rclcpp::Time stamp = msg->header.stamp;
  std::lock_guard<std::mutex> lk(buffer_mutex_);

  auto imu_input = std::make_shared<sensor_msgs::msg::Imu>(*msg);
  if (time_sync_en_ && std::abs(timediff_lidar_wrt_imu_) > static_cast<int64_t>(1.0e8)) {
    stamp += rclcpp::Duration::from_nanoseconds(timediff_lidar_wrt_imu_);
    imu_input->header.stamp = stamp;
  }

  if (stamp < last_imu_timestamp_) {
    RCLCPP_WARN_STREAM(get_logger(),
                       "IMU loopback, clearing buffers (previous: "
                           << last_imu_timestamp_.nanoseconds()
                           << " vs. received: " << stamp.nanoseconds() << " [ns]");
    imu_buffer_.clear();
    kf_for_preintegration_.reset();
  }
  last_imu_timestamp_ = stamp;

  if (kf_for_preintegration_.has_value()) {
    integrateIMU(*kf_for_preintegration_, *imu_input);
  }

  imu_buffer_.push_back(imu_input);
}

void SPARKFastLIO2::integrateIMU(esekfom::esekf<state_ikfom, 12, input_ikfom> &state,
                                 const sensor_msgs::msg::Imu &msg) {
  static std::deque<sensor_msgs::msg::Imu> imu_queue;
  imu_queue.push_back(msg);

  if (imu_queue.size() < 2) {
    return;
  }

  double dt = rclcpp::Time(imu_queue[1].header.stamp).seconds() -
              rclcpp::Time(imu_queue[0].header.stamp).seconds();

  if (dt <= 0) {
    RCLCPP_ERROR(this->get_logger(), "IMU timestamps must be in ascending order!");
    imu_queue.pop_front();
    return;
  }

  core_.integrateIMU(state, imu_queue);
  imu_queue.pop_front();
}

void SPARKFastLIO2::publishOdometry(const state_ikfom &state, const rclcpp::Time &stamp) {
  odomAftMapped_.header.frame_id = map_frame_;
  odomAftMapped_.header.stamp    = stamp;

  setPoseStamp(state, odomAftMapped_.pose, viz_frame_);

  if (viz_frame_ == "lidar") {
    odomAftMapped_.child_frame_id = lidar_frame_;
  } else if (viz_frame_ == "base") {
    odomAftMapped_.child_frame_id = base_frame_;
  } else if (viz_frame_ == "imu") {
    odomAftMapped_.child_frame_id = imu_frame_;
  } else {
    throw std::invalid_argument("Invalid visualization frame has been given");
  }

  // fill the covariance
  auto P = core_.getKf().get_P();
  for (int i = 0; i < 6; i++) {
    int k                                     = (i < 3) ? (i + 3) : (i - 3);
    odomAftMapped_.pose.covariance[i * 6 + 0] = P(k, 3);
    odomAftMapped_.pose.covariance[i * 6 + 1] = P(k, 4);
    odomAftMapped_.pose.covariance[i * 6 + 2] = P(k, 5);
    odomAftMapped_.pose.covariance[i * 6 + 3] = P(k, 0);
    odomAftMapped_.pose.covariance[i * 6 + 4] = P(k, 1);
    odomAftMapped_.pose.covariance[i * 6 + 5] = P(k, 2);
  }

  pub_odom_->publish(odomAftMapped_);

  geometry_msgs::msg::TransformStamped transform_stamped;
  transform_stamped.header.stamp    = odomAftMapped_.header.stamp;
  transform_stamped.header.frame_id = map_frame_;
  transform_stamped.child_frame_id  = odomAftMapped_.child_frame_id;

  transform_stamped.transform.translation.x = odomAftMapped_.pose.pose.position.x;
  transform_stamped.transform.translation.y = odomAftMapped_.pose.pose.position.y;
  transform_stamped.transform.translation.z = odomAftMapped_.pose.pose.position.z;
  transform_stamped.transform.rotation      = odomAftMapped_.pose.pose.orientation;

  tf_broadcaster_->sendTransform(transform_stamped);
}

void SPARKFastLIO2::publishPath(const state_ikfom &state) {
  setPoseStamp(state, msg_body_pose_, viz_frame_);
  msg_body_pose_.header.stamp    = rclcpp::Time(core_.getLidarEndTime() * 1e9);
  msg_body_pose_.header.frame_id = map_frame_;

  static int jjj = 0;
  jjj++;
  if (jjj % 10 == 0) {
    path_msg_.poses.push_back(msg_body_pose_);
    pub_path_->publish(path_msg_);
  }
}

void SPARKFastLIO2::publishDebugData(const state_ikfom &state, const rclcpp::Time &stamp) {
  // 1. IMU preintegration Pose
  if (has_preint_state_) {
    V3D preint_pos_aligned = R_gravity_aligned_ * preint_state_before_update_.pos;
    Eigen::Quaterniond preint_rot_aligned(R_gravity_aligned_ * preint_state_before_update_.rot.toRotationMatrix());

    geometry_msgs::msg::PoseStamped preint_msg;
    preint_msg.header.stamp = stamp;
    preint_msg.header.frame_id = map_frame_;
    preint_msg.pose.position.x = preint_pos_aligned(0);
    preint_msg.pose.position.y = preint_pos_aligned(1);
    preint_msg.pose.position.z = preint_pos_aligned(2);
    preint_msg.pose.orientation.x = preint_rot_aligned.x();
    preint_msg.pose.orientation.y = preint_rot_aligned.y();
    preint_msg.pose.orientation.z = preint_rot_aligned.z();
    preint_msg.pose.orientation.w = preint_rot_aligned.w();
    pub_debug_preint_pose_->publish(preint_msg);
  }

  // 2. Delta Pose
  if (has_last_state_) {
    geometry_msgs::msg::Pose delta_msg;
    V3D delta_pos = state.pos - last_state_.pos;
    delta_msg.position.x = delta_pos(0);
    delta_msg.position.y = delta_pos(1);
    delta_msg.position.z = delta_pos(2);
    Eigen::Quaterniond delta_rot = state.rot * last_state_.rot.inverse();
    delta_msg.orientation.x = delta_rot.x();
    delta_msg.orientation.y = delta_rot.y();
    delta_msg.orientation.z = delta_rot.z();
    delta_msg.orientation.w = delta_rot.w();
    pub_debug_delta_pose_->publish(delta_msg);
  }

  // 3. IMU Bias
  geometry_msgs::msg::Vector3Stamped bias_gyro_msg;
  bias_gyro_msg.header.stamp = stamp;
  bias_gyro_msg.vector.x = state.bg(0);
  bias_gyro_msg.vector.y = state.bg(1);
  bias_gyro_msg.vector.z = state.bg(2);
  pub_debug_bias_gyro_->publish(bias_gyro_msg);

  geometry_msgs::msg::Vector3Stamped bias_acc_msg;
  bias_acc_msg.header.stamp = stamp;
  bias_acc_msg.vector.x = state.ba(0);
  bias_acc_msg.vector.y = state.ba(1);
  bias_acc_msg.vector.z = state.ba(2);
  pub_debug_bias_acc_->publish(bias_acc_msg);

  // 4. Match quality
  std_msgs::msg::Float64MultiArray quality_msg;
  quality_msg.data.resize(4);
  quality_msg.data[0] = static_cast<double>(core_.getEffectFeatNum());
  quality_msg.data[1] = core_.getResMeanLast();

  double imu_res = 0.0;
  if (has_preint_state_) {
    V3D preint_pos_aligned = R_gravity_aligned_ * preint_state_before_update_.pos;
    Eigen::Quaterniond preint_rot_aligned(R_gravity_aligned_ * preint_state_before_update_.rot.toRotationMatrix());

    V3D pos_diff = preint_pos_aligned - state.pos;
    double pos_res = pos_diff.norm();
    Eigen::Quaterniond rot_diff = preint_rot_aligned * state.rot.inverse();
    double rot_res = 2.0 * std::acos(std::clamp(std::abs(rot_diff.w()), 0.0, 1.0));
    imu_res = pos_res + rot_res;
  }
  quality_msg.data[2] = 0.0;  // solve_time not exposed
  quality_msg.data[3] = imu_res;
  pub_debug_quality_->publish(quality_msg);

  // 5. Velocity
  geometry_msgs::msg::Vector3Stamped vel_msg;
  vel_msg.header.stamp = stamp;
  vel_msg.vector.x = state.vel(0);
  vel_msg.vector.y = state.vel(1);
  vel_msg.vector.z = state.vel(2);
  pub_debug_velocity_->publish(vel_msg);

  last_state_ = state;
  has_last_state_ = true;
}

void SPARKFastLIO2::publishFrameWorld(
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubCloud) {
  if (!scan_pub_en_) {
    return;
  }

  const auto &cloud_undistort = core_.getCloudUndistort();
  const auto &feats_down_body = core_.getFeatsDownBody();

  // choose which cloud to publish
  PointCloudXYZI::Ptr laserCloudFullRes(dense_pub_en_ ? cloud_undistort : feats_down_body);

  int size = laserCloudFullRes->points.size();
  PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));
  PointCloudXYZI::Ptr laserCloudTmp(new PointCloudXYZI(size, 1));

  for (int i = 0; i < size; i++) {
    if (viz_frame_ == "imu") {
      pclPointBodyToWorld(&laserCloudFullRes->points[i], &laserCloudWorld->points[i]);
    } else if (viz_frame_ == "lidar") {
      pclPointBodyToWorld(&laserCloudFullRes->points[i], &laserCloudTmp->points[i]);
      pclPointIMUToLiDAR(&laserCloudTmp->points[i], &laserCloudWorld->points[i]);
    } else if (viz_frame_ == "base") {
      pclPointBodyToWorld(&laserCloudFullRes->points[i], &laserCloudTmp->points[i]);
      pclPointIMUToBase(&laserCloudTmp->points[i], &laserCloudWorld->points[i]);
    } else {
      throw std::invalid_argument("Invalid visualization frame has been given");
    }
  }

  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(*laserCloudWorld, cloud_msg);
  cloud_msg.header.stamp    = rclcpp::Time(core_.getLidarEndTime() * 1e9);
  cloud_msg.header.frame_id = map_frame_;

  pubCloud->publish(cloud_msg);
  publish_count_ -= PUBFRAME_PERIOD;

  if (pcd_save_en_) {
    int nsize = cloud_undistort->points.size();
    PointCloudXYZI::Ptr laserCloudWorld2(new PointCloudXYZI(nsize, 1));

    for (int i = 0; i < nsize; i++) {
      pclPointBodyToWorld(&cloud_undistort->points[i], &laserCloudWorld2->points[i]);
    }
    if (pcd_save_interval_ > 0) {
      *cloud_to_be_saved_ += *laserCloudWorld2;
    }

    static int scan_wait_num = 0;
    scan_wait_num++;
    if (cloud_to_be_saved_->size() > 0 && pcd_save_interval_ > 0 &&
        scan_wait_num >= pcd_save_interval_) {
      pcd_index_++;
      std::string all_points_dir(std::string(ROOT_DIR) + "PCD/scans_" + std::to_string(pcd_index_) +
                                 ".pcd");
      pcl::PCDWriter pcd_writer;
      std::cout << "Current scan saved to /PCD/ " << all_points_dir << std::endl;
      pcd_writer.writeBinary(all_points_dir, *cloud_to_be_saved_);
      cloud_to_be_saved_->clear();
      scan_wait_num = 0;
    }
  }
}

void SPARKFastLIO2::publishFrame(
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubCloud,
    const std::string &frame) {
  const auto &cloud_undistort = core_.getCloudUndistort();
  int size = cloud_undistort->points.size();
  PointCloudXYZI::Ptr laserCloudTransformed(new PointCloudXYZI(size, 1));
  sensor_msgs::msg::PointCloud2 cloud_msg;

  if (frame == "lidar") {
    for (int i = 0; i < size; i++) {
      laserCloudTransformed->points[i] = cloud_undistort->points[i];
    }
    pcl::toROSMsg(*laserCloudTransformed, cloud_msg);
    cloud_msg.header.stamp    = rclcpp::Time(core_.getLidarEndTime() * 1e9);
    cloud_msg.header.frame_id = lidar_frame_;
  } else if (frame == "imu") {
    for (int i = 0; i < size; i++) {
      pclPointBodyLidarToIMU(&cloud_undistort->points[i], &laserCloudTransformed->points[i]);
    }
    pcl::toROSMsg(*laserCloudTransformed, cloud_msg);
    cloud_msg.header.stamp    = rclcpp::Time(core_.getLidarEndTime() * 1e9);
    cloud_msg.header.frame_id = imu_frame_;
  } else if (frame == "base") {
    for (int i = 0; i < size; i++) {
      pclPointBodyLidarToBase(&cloud_undistort->points[i], &laserCloudTransformed->points[i]);
    }
    pcl::toROSMsg(*laserCloudTransformed, cloud_msg);
    cloud_msg.header.stamp    = rclcpp::Time(core_.getLidarEndTime() * 1e9);
    cloud_msg.header.frame_id = base_frame_;
  } else {
    throw std::invalid_argument("Invalid frame has been given");
  }

  pubCloud->publish(cloud_msg);
  publish_count_ -= PUBFRAME_PERIOD;
}

PoseStruct SPARKFastLIO2::transformPoseWrtLidarFrame(const state_ikfom &state) const {
  Eigen::Vector3d lidar_position = state.offset_R_L_I.inverse() * (state.rot * state.offset_T_L_I +
                                                                   state.pos - state.offset_T_L_I);

  Eigen::Quaterniond lidar_orientation =
      state.offset_R_L_I.inverse() * state.rot * state.offset_R_L_I;

  PoseStruct output;
  output.position_    = lidar_position;
  output.orientation_ = lidar_orientation;
  return output;
}

void SPARKFastLIO2::main() {
  if (syncPackages(Measures_, verbose_)) {
    processLidarAndImu(Measures_);
  }
}

PoseStruct SPARKFastLIO2::transformPoseWrtBaseFrame(const state_ikfom &state) const {
  const Eigen::Matrix3d offset_R_B_I = state.offset_R_L_I * lidar_R_wrt_base_.inverse();
  const Eigen::Vector3d offset_T_B_I =
      -offset_R_B_I * lidar_T_wrt_base_ + state.offset_T_L_I;

  Eigen::Vector3d base_position =
      offset_R_B_I.inverse() * (state.rot * offset_T_B_I + state.pos - offset_T_B_I);

  Eigen::Quaterniond base_orientation =
      Eigen::Quaterniond(offset_R_B_I.inverse() * state.rot * offset_R_B_I);

  PoseStruct output;
  output.position_    = base_position;
  output.orientation_ = base_orientation;
  return output;
}

bool SPARKFastLIO2::syncPackages(MeasureGroup &meas, bool verbose) {
  std::lock_guard<std::mutex> lk(buffer_mutex_);
  if (verbose) {
    static size_t num_lidar_prev = 0;
    static size_t num_imu_prev   = 0;
    size_t num_lidar_curr        = lidar_buffer_.size();
    size_t num_imu_curr          = imu_buffer_.size();

    if ((num_lidar_prev != num_lidar_curr) || (num_imu_prev != num_imu_curr)) {
      RCLCPP_INFO(this->get_logger(), "%lu vs. %lu", num_lidar_curr, num_imu_curr);
      num_lidar_prev = num_lidar_curr;
      num_imu_prev   = num_imu_curr;
    }
  }

  if (lidar_buffer_.empty() || imu_buffer_.empty()) return false;

  if (!lidar_pushed_) {
    meas.lidar                = lidar_buffer_.front();
    meas.lidar_beg_time       = time_buffer_.front();
    static double denominator = 1000;

    if (meas.lidar->points.size() <= 1) {
      lidar_end_time_ = meas.lidar_beg_time + lidar_mean_scantime_;
    } else if (meas.lidar->points.back().curvature / denominator < 0.5 * lidar_mean_scantime_) {
      lidar_end_time_ = meas.lidar_beg_time + lidar_mean_scantime_;
    } else {
      scan_num_++;
      if (meas.lidar->points.back().curvature < 80 || meas.lidar->points.back().curvature > 120) {
        RCLCPP_WARN(this->get_logger(),
                    "meas.lidar->points.back().curvature (%.2f) should be close to 100. Please "
                    "check the `timestamp_unit` "
                    "or values of `time` (or `t`) field of the point cloud input from your sensor.",
                    meas.lidar->points.back().curvature);
      }

      double dt        = meas.lidar->points.back().curvature / 1000.0;
      lidar_end_time_  = meas.lidar_beg_time + dt;
      lidar_mean_scantime_ += (dt - lidar_mean_scantime_) / static_cast<double>(scan_num_);
    }
    meas.lidar_end_time = lidar_end_time_;
    lidar_pushed_       = true;
  }

  if (last_imu_timestamp_.seconds() < lidar_end_time_) {
    if (verbose) {
      static rclcpp::Time last_imu_timestamp_prev(now());
      if (last_imu_timestamp_prev != last_imu_timestamp_) {
        RCLCPP_INFO(this->get_logger(),
                    "Not enough IMU data (%.6f < %.6f)",
                    last_imu_timestamp_.seconds(),
                    lidar_end_time_);
        last_imu_timestamp_prev = last_imu_timestamp_;
      }
    }
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

bool SPARKFastLIO2::isMotionStopped(const V3D &acc_ref,
                                    const V3D &acc_curr,
                                    const double acc_diff_thr) {
  return (acc_ref - acc_curr).norm() <= acc_diff_thr;
}

void SPARKFastLIO2::processLidarAndImu(MeasureGroup &Measures) {
  // Delegate algorithm to core
  core_.processLidarAndImu(Measures);

  // If first scan, core returns early - nothing to publish
  if (core_.isFirstScan()) {
    return;
  }

  const auto &latest_state = core_.getLatestState();

  // Check if we have valid features
  if (core_.getFeatsDownSize() == 0) {
    return;
  }

  // Gravity alignment (ROS-layer only)
  static int num_consecutive_moving_frames = 0;
  if (enable_gravity_alignment_ && !is_gravity_aligned_ && !base_frame_.empty()) {
    if (!core_.isEKFInitialized()) {
      mean_acc_stopped_ = Measures.getMeanAcc();
    } else {
      const auto &mean_acc = Measures.getMeanAcc();
      if (isMotionStopped(mean_acc_stopped_, mean_acc, acc_diff_thr_)) {
        RCLCPP_WARN_STREAM(
            this->get_logger(),
            "Waiting for motion to perform gravity alignment...now a robot has been stopped");
        num_consecutive_moving_frames = 0;
      } else {
        num_consecutive_moving_frames = min(num_consecutive_moving_frames + 1, 100000);
      }
    }
  }

  // Save preintegration state for debug
  preint_state_before_update_ = core_.getPredictionState();
  has_preint_state_ = true;

  // Perform gravity alignment
  if (enable_gravity_alignment_ && !is_gravity_aligned_ && !base_frame_.empty() &&
      (num_consecutive_moving_frames > num_moving_frames_thr_)) {
    const auto &latest_state_ref = core_.getLatestState();
    static const auto &offset_R_I_B = lidar_R_wrt_base_ * latest_state_ref.offset_R_L_I.inverse();

    V3D gravity_direction = core_.getUpdateState().grav;
    if (global_gravity_directions_.size() < static_cast<size_t>(num_gravity_measurements_thr_)) {
      {
        std::stringstream ss;
        ss << "Waiting for motion: " << global_gravity_directions_.size() << " / "
           << num_gravity_measurements_thr_;
        RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
      }

      global_gravity_directions_.push_back(offset_R_I_B * gravity_direction);
    } else {
      V3D avg_global_gravity_vec = Eigen::Vector3d::Zero();
      for (const auto &gravity_vec : global_gravity_directions_) {
        avg_global_gravity_vec += gravity_vec;
      }
      avg_global_gravity_vec /= global_gravity_directions_.size();

      R_gravity_aligned_ = computeRelativeRotation(avg_global_gravity_vec, g_base_);

      {
        std::stringstream ss;
        ss << "Gravity alignment complete! `R_gravity_aligned`: " << R_gravity_aligned_;
        RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
      }

      is_gravity_aligned_ = true;
    }
  }

  // Apply gravity alignment to the state for publishing
  state_ikfom aligned_state = core_.getLatestState();
  if (is_gravity_aligned_) {
    aligned_state.pos = R_gravity_aligned_ * aligned_state.pos;
    aligned_state.rot = R_gravity_aligned_ * aligned_state.rot;
  }

  // Update kf_for_preintegration_ for IMU callback
  kf_for_preintegration_ = core_.getKf();

  if (enable_gravity_alignment_ && !is_gravity_aligned_ && !base_frame_.empty()) {
    RCLCPP_WARN(this->get_logger(),
                "Gravity alignment is enabled but not yet completed. Waiting for alignment...");
    return;
  }

  /******* Publish topics *******/
  const auto stamp = rclcpp::Time(core_.getLidarEndTime() * 1e9);
  publishOdometry(aligned_state, stamp);
  publishDebugData(aligned_state, stamp);

  if (path_en_) {
    publishPath(aligned_state);
  }
  if (scan_pub_en_) {
    publishFrameWorld(pub_cloud_full_);
    if (scan_lidar_pub_en_) publishFrame(pub_cloud_lidar_, "lidar");
    if (scan_body_pub_en_) publishFrame(pub_cloud_body_, "imu");
    if (scan_base_pub_en_) publishFrame(pub_cloud_base_, "base");
  }
}
}  // namespace spark_fast_lio

RCLCPP_COMPONENTS_REGISTER_NODE(spark_fast_lio::SPARKFastLIO2)