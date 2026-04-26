#pragma once

#include <deque>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "common/so3_math.h"
#include "common/common_lib.h"
#include "fast_lio2_core.hpp"
#include "preprocess.h"

#define MAXN (720000)
#define PUBFRAME_PERIOD (20)

#if defined(LIVOX_ROS_DRIVER_FOUND) && LIVOX_ROS_DRIVER_FOUND
#include <livox_ros_driver2/msg/custom_msg.hpp>
#endif

namespace spark_fast_lio {

struct PoseStruct {
  Eigen::Vector3d position_;
  Eigen::Quaterniond orientation_;
};

class SPARKFastLIO2 : public rclcpp::Node {
 public:
  explicit SPARKFastLIO2(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

 private:
  M3D computeRelativeRotation(const Eigen::Vector3d &g_a, const Eigen::Vector3d &g_b);

  bool lookupBaseExtrinsics(V3D &lidar_T_wrt_base, M3D &lidar_R_wrt_base);

  void pclPointBodyToWorld(PointType const *const pi, PointType *const po);
  void pclPointBodyLidarToIMU(PointType const *const pi, PointType *const po);
  void pclPointBodyLidarToBase(PointType const *const pi, PointType *const po);
  void pclPointIMUToLiDAR(PointType const *const pi, PointType *const po);
  void pclPointIMUToBase(PointType const *const pi, PointType *const po);

  void standardLiDARCallback(const sensor_msgs::msg::PointCloud2 &msg);

#if defined(LIVOX_ROS_DRIVER_FOUND) && LIVOX_ROS_DRIVER_FOUND
  void livoxLiDARCallback(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr msg);
#endif

  void imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr msg);

  void integrateIMU(esekfom::esekf<state_ikfom, 12, input_ikfom> &state,
                    const sensor_msgs::msg::Imu &msg);

  void publishOdometry(const state_ikfom &state, const rclcpp::Time &stamp);
  void publishPath(const state_ikfom &state);
  void publishFrameWorld(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubCloud);
  void publishFrame(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubCloud,
                    const std::string &frame);
  void publishDebugData(const state_ikfom &state, const rclcpp::Time &stamp);

  PoseStruct transformPoseWrtLidarFrame(const state_ikfom &state) const;
  PoseStruct transformPoseWrtBaseFrame(const state_ikfom &state) const;

  template <typename T>
  void setPoseStamp(const state_ikfom &state, T &out, const std::string &frame) const {
    if (frame == "imu") {
      out.pose.position.x    = state.pos(0);
      out.pose.position.y    = state.pos(1);
      out.pose.position.z    = state.pos(2);
      const auto quat        = state.rot.coeffs();
      out.pose.orientation.x = quat[0];
      out.pose.orientation.y = quat[1];
      out.pose.orientation.z = quat[2];
      out.pose.orientation.w = quat[3];
    } else if (frame == "lidar") {
      const auto &p          = transformPoseWrtLidarFrame(state);
      out.pose.position.x    = p.position_(0);
      out.pose.position.y    = p.position_(1);
      out.pose.position.z    = p.position_(2);
      out.pose.orientation.x = p.orientation_.x();
      out.pose.orientation.y = p.orientation_.y();
      out.pose.orientation.z = p.orientation_.z();
      out.pose.orientation.w = p.orientation_.w();
    } else if (frame == "base") {
      const auto &p          = transformPoseWrtBaseFrame(state);
      out.pose.position.x    = p.position_(0);
      out.pose.position.y    = p.position_(1);
      out.pose.position.z    = p.position_(2);
      out.pose.orientation.x = p.orientation_.x();
      out.pose.orientation.y = p.orientation_.y();
      out.pose.orientation.z = p.orientation_.z();
      out.pose.orientation.w = p.orientation_.w();
    } else {
      throw std::invalid_argument("Invalid visualization frame has been given");
    }
  }

  void main();
  bool syncPackages(MeasureGroup &meas, bool verbose);
  bool isMotionStopped(const V3D &acc_ref, const V3D &acc_curr, const double acc_diff_thr);
  void processLidarAndImu(MeasureGroup &Measure);

  // ========== Algorithm core (composition) ==========
  FastLIO2Core core_;

  // ========== ROS-specific members ==========
  std::mutex buffer_mutex_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar_;

#if defined(LIVOX_ROS_DRIVER_FOUND) && LIVOX_ROS_DRIVER_FOUND
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_lidar_livox_;
#endif

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_full_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_lidar_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_body_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_base_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;

  // Debug publishers
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_debug_preint_pose_;
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_debug_delta_pose_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_debug_bias_gyro_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_debug_bias_acc_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_debug_quality_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_debug_velocity_;

  // Debug state (ROS-layer only)
  state_ikfom last_state_;
  state_ikfom preint_state_before_update_;
  bool has_last_state_ = false;
  bool has_preint_state_ = false;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Clock::SharedPtr clock_;
  rclcpp::TimerBase::SharedPtr main_loop_timer_;

  // ROS parameters
  bool pcd_save_en_       = false;
  bool time_sync_en_      = false;
  bool path_en_           = true;
  bool scan_pub_en_       = false;
  bool dense_pub_en_      = false;
  bool scan_lidar_pub_en_ = false;
  bool scan_body_pub_en_  = false;
  bool scan_base_pub_en_  = false;
  bool verbose_           = false;
  bool pcl_verbose_       = true;
  bool runtime_pos_log_   = false;

  // Gravity alignment (ROS-layer only)
  bool enable_gravity_alignment_ = false;
  bool is_gravity_aligned_       = false;
  double acc_diff_thr_           = 0.2;
  int num_moving_frames_thr_     = 10;
  int num_gravity_measurements_thr_ = 10;
  std::deque<V3D> global_gravity_directions_;
  V3D g_base_;
  V3D mean_acc_stopped_;
  M3D R_gravity_aligned_;

  // Extrinsics for base frame (ROS-layer only)
  V3D lidar_T_wrt_base_;
  M3D lidar_R_wrt_base_;
  double extrinsics_timeout_s_ = 10.0;

  // Frame names
  std::string map_frame_;
  std::string lidar_frame_;
  std::string base_frame_;
  std::string imu_frame_;
  std::string viz_frame_;

  // PCD save
  int pcd_save_interval_ = -1;
  int pcd_index_         = 0;
  PointCloudXYZI::Ptr cloud_to_be_saved_;

  // Data buffers (ROS-layer only, for syncPackages)
  rclcpp::Time last_lidar_timestamp_;
  rclcpp::Time last_imu_timestamp_;
  int64_t timediff_lidar_wrt_imu_ = 0;
  bool lidar_pushed_ = false;
  double lidar_mean_scantime_ = 0.0;
  double lidar_end_time_ = 0.0;  // computed in syncPackages, passed to core via MeasureGroup
  int scan_num_ = 0;
  int scan_count_ = 0;
  int publish_count_ = 0;
  std::deque<double> time_buffer_;
  std::deque<PointCloudXYZI::Ptr> lidar_buffer_;
  std::deque<std::shared_ptr<const sensor_msgs::msg::Imu>> imu_buffer_;
  MeasureGroup Measures_;

  // Preprocessor (ROS-layer only, for lidar callback)
  std::shared_ptr<Preprocess> preprocessor_;

  // ESKF for preintegration (ROS-layer only, for IMU callback)
  std::optional<esekfom::esekf<state_ikfom, 12, input_ikfom>> kf_for_preintegration_;

  // ROS messages
  nav_msgs::msg::Path path_msg_;
  nav_msgs::msg::Odometry odomAftMapped_;
  geometry_msgs::msg::PoseStamped msg_body_pose_;

  // Extrinsics config
  std::vector<double> extrinT_{0.0, 0.0, 0.0};
  std::vector<double> extrinR_{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
};

}  // namespace spark_fast_lio