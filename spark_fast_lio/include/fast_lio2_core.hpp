/**
 * @file fast_lio2_core.hpp
 * @brief Fast-LIO2 algorithm core, independent of ROS.
 *
 * This class contains the ONLY algorithm logic. Both SPARKFastLIO2 (ROS node)
 * and run_sdk (offline runner) use this class via composition.
 */

#pragma once

#include <vector>
#include <memory>
#include <functional>

#include <Eigen/Core>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "common/so3_math.h"
#include "ikd_Tree.h"
#include "imu_processing.hpp"
#include "common/use-ikfom.hpp"

#define INIT_TIME (0.1)
#define LASER_POINT_COV (0.001)
#define NUM_MATCH_POINTS (5)
#define MOV_THRESHOLD (1.5)

namespace spark_fast_lio {

using PointType = pcl::PointXYZINormal;
using PointCloudXYZI = pcl::PointCloud<PointType>;
using PointVector = std::vector<PointType, Eigen::aligned_allocator<PointType>>;
using V3D = Eigen::Vector3d;
using M3D = Eigen::Matrix3d;

class FastLIO2Core {
 public:
  struct Config {
    int point_filter_num = 4;
    double filter_size_map_min = 0.5;
    double cube_len = 200.0;
    double det_range = 300.0;
    int max_iterations = 4;
    bool extrinsic_est_en = false;
    bool verbose = false;

    // IMU parameters
    V3D gyr_cov = V3D(0.1, 0.1, 0.1);
    V3D acc_cov = V3D(0.1, 0.1, 0.1);
    V3D gyr_bias_cov = V3D(0.0001, 0.0001, 0.0001);
    V3D acc_bias_cov = V3D(0.0001, 0.0001, 0.0001);

    // Extrinsics
    V3D lidar_T_wrt_imu = V3D::Zero();
    M3D lidar_R_wrt_imu = M3D::Identity();
  };

  FastLIO2Core() : FastLIO2Core(Config{}) {}
  FastLIO2Core(const Config &config);
  ~FastLIO2Core() = default;

  /** @brief Core algorithm: process one MeasureGroup */
  void processLidarAndImu(MeasureGroup &meas);

  // --- Getters for pred/update states/covariances ---
  const state_ikfom &getPredictionState() const { return pred_state_; }
  const Eigen::Matrix<double, 23, 23> &getPredictionCovariance() const { return pred_cov_; }
  const state_ikfom &getUpdateState() const { return update_state_; }
  const Eigen::Matrix<double, 23, 23> &getUpdateCovariance() const { return update_cov_; }
  const state_ikfom &getLatestState() const { return latest_state_; }

  // --- Getters for internal state needed by ROS layer ---
  bool isEKFInitialized() const { return flg_EKF_inited_; }
  bool isFirstScan() const { return flg_first_scan_; }
  int getFeatsDownSize() const { return feats_down_size_; }
  int getEffectFeatNum() const { return effect_feat_num_; }
  double getResMeanLast() const { return res_mean_last_; }
  double getLidarEndTime() const { return lidar_end_time_; }
  const PointCloudXYZI::Ptr &getCloudUndistort() const { return cloud_undistort_; }
  const PointCloudXYZI::Ptr &getFeatsDownBody() const { return feats_down_body_; }
  const esekfom::esekf<state_ikfom, 12, input_ikfom> &getKf() const { return kf_; }

  // --- Setters for gravity alignment (applied by ROS layer after update) ---
  void applyGravityAlignment(const M3D &R_gravity_aligned);

  // --- IMU preintegration for external use (e.g., ROS debug publishing) ---
  void integrateIMU(esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state,
                    const std::deque<sensor_msgs::msg::Imu> &imu_queue);

 private:
  void pointBodyToWorld(const PointType *pi, PointType *po, const state_ikfom &s);
  void calcHModel(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data);
  bool esti_plane(Eigen::Vector4f &pca_result, const PointVector &point, const float &threshold);
  void lasermapFovSegment();
  void mapIncremental();

 private:
  Config config_;

  // ESKF
  esekfom::esekf<state_ikfom, 12, input_ikfom> kf_;
  state_ikfom latest_state_;

  // Prediction and update states/covariances
  state_ikfom pred_state_;
  Eigen::Matrix<double, 23, 23> pred_cov_;
  state_ikfom update_state_;
  Eigen::Matrix<double, 23, 23> update_cov_;

  // IMU processor
  std::shared_ptr<ImuProcess> imu_processor_;

  // Point clouds
  PointCloudXYZI::Ptr cloud_undistort_;
  PointCloudXYZI::Ptr feats_undistort_;
  PointCloudXYZI::Ptr feats_down_body_;
  PointCloudXYZI::Ptr feats_down_world_;
  PointCloudXYZI::Ptr normvec_;
  PointCloudXYZI::Ptr laser_cloud_ori_;
  PointCloudXYZI::Ptr corr_normvec_;

  // ikd-tree
  KD_TREE<PointType> ikd_tree_;
  std::vector<PointVector> nearest_points_;
  std::vector<BoxPointType> cub_needrm_;
  BoxPointType localmap_points_;

  // Filters
  pcl::VoxelGrid<PointType> down_size_filter_;

  // State flags & counters
  bool flg_first_scan_ = true;
  bool flg_EKF_inited_ = false;
  double first_lidar_time_ = 0.0;
  double lidar_end_time_ = 0.0;
  int feats_down_size_ = 0;
  int effect_feat_num_ = 0;
  double res_mean_last_ = 0.05;
  double total_residual_ = 0.0;
  double solve_time_ = 0.0;
  double match_time_ = 0.0;
  int kdtree_delete_counter_ = 0;
  double kdtree_delete_time_ = 0.0;
  int kdtree_size_st_ = 0;
  int add_point_size_ = 0;
  double kdtree_incremental_time_ = 0.0;

  // Feature selection
  std::vector<bool> point_selected_surf_;
  std::vector<double> res_last_;
};

}  // namespace spark_fast_lio