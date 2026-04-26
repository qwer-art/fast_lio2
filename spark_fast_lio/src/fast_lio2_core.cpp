/**
 * @file fast_lio2_core.cpp
 * @brief Fast-LIO2 algorithm core implementation
 */

#include "fast_lio2_core.hpp"

#include <omp.h>

namespace spark_fast_lio {

FastLIO2Core::FastLIO2Core(const Config &config) : config_(config) {
  // Initialize point clouds
  cloud_undistort_.reset(new PointCloudXYZI());
  feats_undistort_.reset(new PointCloudXYZI());
  feats_down_body_.reset(new PointCloudXYZI());
  feats_down_world_.reset(new PointCloudXYZI());
  normvec_.reset(new PointCloudXYZI());
  laser_cloud_ori_.reset(new PointCloudXYZI());
  corr_normvec_.reset(new PointCloudXYZI());

  // Initialize IMU processor
  imu_processor_ = std::make_shared<ImuProcess>();

  // Initialize downsample filter
  down_size_filter_.setLeafSize(config_.filter_size_map_min,
                                 config_.filter_size_map_min,
                                 config_.filter_size_map_min);

  // Set IMU processor parameters
  imu_processor_->set_gyr_cov(config_.gyr_cov);
  imu_processor_->set_acc_cov(config_.acc_cov);
  imu_processor_->set_gyr_bias_cov(config_.gyr_bias_cov);
  imu_processor_->set_acc_bias_cov(config_.acc_bias_cov);
  imu_processor_->set_extrinsic(config_.lidar_T_wrt_imu, config_.lidar_R_wrt_imu);

  // Initialize ESKF
  double epsi[23];
  for (int i = 0; i < 23; ++i) epsi[i] = 0.001;

  kf_.init_dyn_share(
      get_f,
      df_dx,
      df_dw,
      std::bind(&FastLIO2Core::calcHModel, this, std::placeholders::_1, std::placeholders::_2),
      config_.max_iterations,
      epsi);
}

void FastLIO2Core::processLidarAndImu(MeasureGroup &meas) {
  lidar_end_time_ = meas.lidar_end_time;

  if (flg_first_scan_) {
    first_lidar_time_ = meas.lidar_beg_time;
    imu_processor_->first_lidar_time = first_lidar_time_;
    flg_first_scan_ = false;
    return;
  }

  // 1. IMU Processing and Prediction
  cloud_undistort_->clear();
  feats_undistort_->clear();

  imu_processor_->Process(meas, kf_, cloud_undistort_);

  // Resample point cloud
  feats_undistort_->reserve(cloud_undistort_->size() / config_.point_filter_num);
  for (size_t i = 0; i < cloud_undistort_->points.size(); i++) {
    if (i % config_.point_filter_num == 0) {
      feats_undistort_->push_back(cloud_undistort_->points[i]);
    }
  }

  latest_state_ = kf_.get_x();

  if (feats_undistort_->empty()) {
    if (config_.verbose) {
      std::cerr << "[FastLIO2] No point, skip this scan!" << std::endl;
    }
    return;
  }

  // 2. Check EKF initialization
  flg_EKF_inited_ =
      (meas.lidar_beg_time - first_lidar_time_) < INIT_TIME ? false : true;

  // 3. Laser map FOV segment
  lasermapFovSegment();

  // 4. Downsample
  down_size_filter_.setInputCloud(feats_undistort_);
  down_size_filter_.filter(*feats_down_body_);
  feats_down_size_ = feats_down_body_->points.size();

  // 5. Initialize ikd-tree if needed
  if (ikd_tree_.Root_Node == nullptr) {
    if (feats_down_size_ > 5) {
      ikd_tree_.set_downsample_param(config_.filter_size_map_min);
      feats_down_world_->resize(feats_down_size_);
      for (int i = 0; i < feats_down_size_; i++) {
        pointBodyToWorld(&(feats_down_body_->points[i]),
                         &(feats_down_world_->points[i]),
                         latest_state_);
      }
      ikd_tree_.Build(feats_down_world_->points);
    }
    return;
  }

  // 6. ICP and ESKF update
  if (feats_down_size_ < 5) {
    if (config_.verbose) {
      std::cerr << "[FastLIO2] No point, skip this scan!" << std::endl;
    }
    return;
  }

  normvec_->resize(feats_down_size_);
  feats_down_world_->resize(feats_down_size_);
  nearest_points_.resize(feats_down_size_);
  point_selected_surf_.resize(feats_down_size_, true);
  res_last_.resize(feats_down_size_, 0.0);

  // Save prediction state (before ESKF update)
  pred_state_ = kf_.get_x();
  pred_cov_ = kf_.get_P();

  // ESKF update with measurement model
  double solve_H_time = 0;
  kf_.update_iterated_dyn_share_modified(LASER_POINT_COV, solve_H_time);

  // Get updated state
  latest_state_ = kf_.get_x();
  update_state_ = latest_state_;
  update_cov_ = kf_.get_P();

  // 7. Map incremental update
  mapIncremental();
}

void FastLIO2Core::pointBodyToWorld(const PointType *pi, PointType *po,
                                     const state_ikfom &s) {
  V3D p_body(pi->x, pi->y, pi->z);
  V3D p_global(s.rot * (s.offset_R_L_I * p_body + s.offset_T_L_I) + s.pos);
  po->x = p_global(0);
  po->y = p_global(1);
  po->z = p_global(2);
  po->intensity = pi->intensity;
}

void FastLIO2Core::calcHModel(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data) {
  laser_cloud_ori_->clear();
  laser_cloud_ori_->resize(feats_down_size_);
  corr_normvec_->clear();
  corr_normvec_->resize(feats_down_size_);
  double total_residual = 0.0;

  /** closest surface search and residual computation **/
  for (int i = 0; i < feats_down_size_; i++) {
    PointType &point_body  = feats_down_body_->points[i];
    PointType &point_world = feats_down_world_->points[i];

    /* transform to world frame */
    V3D p_body(point_body.x, point_body.y, point_body.z);
    V3D p_global(s.rot * (s.offset_R_L_I * p_body + s.offset_T_L_I) + s.pos);
    point_world.x         = p_global(0);
    point_world.y         = p_global(1);
    point_world.z         = p_global(2);
    point_world.intensity = point_body.intensity;

    std::vector<float> pointSearchSqDis(NUM_MATCH_POINTS);

    auto &points_near = nearest_points_[i];

    if (ekfom_data.converge) {
      /** Find the closest surfaces in the map **/
      ikd_tree_.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);
      point_selected_surf_[i] = points_near.size() < NUM_MATCH_POINTS        ? false
                                : pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5 ? false
                                                                             : true;
    }

    if (!point_selected_surf_[i]) continue;

    Eigen::Vector4f pabcd;
    point_selected_surf_[i] = false;
    if (esti_plane(pabcd, points_near, 0.1f)) {
      float pd2 =
          pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z + pabcd(3);
      float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());

      if (s > 0.9) {
        point_selected_surf_[i]       = true;
        normvec_->points[i].x         = pabcd(0);
        normvec_->points[i].y         = pabcd(1);
        normvec_->points[i].z         = pabcd(2);
        normvec_->points[i].intensity = pd2;
        res_last_[i]                  = abs(pd2);
      }
    }
  }

  int effect_feat_num = 0;

  for (int i = 0; i < feats_down_size_; i++) {
    if (point_selected_surf_[i]) {
      laser_cloud_ori_->points[effect_feat_num] = feats_down_body_->points[i];
      corr_normvec_->points[effect_feat_num]    = normvec_->points[i];
      total_residual += res_last_[i];
      effect_feat_num++;
    }
  }

  if (effect_feat_num < 1) {
    ekfom_data.valid = false;
    if (config_.verbose) {
      std::cerr << "[FastLIO2] No Effective Points!" << std::endl;
    }
    return;
  }

  /*** Computation of Measurement Jacobian matrix H and measurements vector ***/
  ekfom_data.h_x = Eigen::MatrixXd::Zero(effect_feat_num, 12);  // 23
  ekfom_data.h.resize(effect_feat_num);

  for (int i = 0; i < effect_feat_num; i++) {
    const PointType &laser_p = laser_cloud_ori_->points[i];
    V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
    M3D point_be_crossmat;
    point_be_crossmat << SKEW_SYM_MATRX(point_this_be);
    V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I;
    M3D point_crossmat;
    point_crossmat << SKEW_SYM_MATRX(point_this);

    /*** get the normal vector of closest surface/corner ***/
    const PointType &norm_p = corr_normvec_->points[i];
    V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

    /*** calculate the Measurement Jacobian matrix H ***/
    V3D C(s.rot.conjugate() * norm_vec);
    V3D A(point_crossmat * C);
    if (config_.extrinsic_est_en) {
      V3D B(point_be_crossmat * s.offset_R_L_I.conjugate() * C);
      ekfom_data.h_x.block<1, 12>(i, 0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A),
          VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
    } else {
      ekfom_data.h_x.block<1, 12>(i, 0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0;
    }

    /*** Measurement: distance to the closest surface/corner ***/
    ekfom_data.h(i) = -norm_p.intensity;
  }
}

bool FastLIO2Core::esti_plane(Eigen::Vector4f &pca_result, const PointVector &point, const float &threshold) {
  Eigen::Matrix<float, NUM_MATCH_POINTS, 3> A;
  Eigen::Matrix<float, NUM_MATCH_POINTS, 1> b;
  A.setZero();
  b.setOnes();
  b *= -1.0f;

  for (int j = 0; j < NUM_MATCH_POINTS; j++) {
    A(j, 0) = point[j].x;
    A(j, 1) = point[j].y;
    A(j, 2) = point[j].z;
  }

  Eigen::Vector3f normvec = A.colPivHouseholderQr().solve(b);

  pca_result(0) = normvec(0);
  pca_result(1) = normvec(1);
  pca_result(2) = normvec(2);
  pca_result(3) = 1.0 / normvec.norm();

  // Check if points are close to the plane
  for (int j = 0; j < NUM_MATCH_POINTS; j++) {
    if (fabs(pca_result(0) * point[j].x + pca_result(1) * point[j].y + pca_result(2) * point[j].z +
             pca_result(3)) > threshold) {
      return false;
    }
  }
  return true;
}

void FastLIO2Core::lasermapFovSegment() {
  static bool localmap_initialized = false;

  cub_needrm_.clear();
  kdtree_delete_counter_ = 0;
  kdtree_delete_time_ = 0.0;

  V3D lidar_xyz = kf_.get_lidar_position();

  if (!localmap_initialized) {
    for (int i = 0; i < 3; i++) {
      localmap_points_.vertex_min[i] = lidar_xyz(i) - config_.cube_len / 2.0;
      localmap_points_.vertex_max[i] = lidar_xyz(i) + config_.cube_len / 2.0;
    }
    localmap_initialized = true;
    return;
  }

  float dist_to_map_edge[3][2];
  bool need_move = false;

  for (int i = 0; i < 3; i++) {
    dist_to_map_edge[i][0] =
        fabs(lidar_xyz(i) - localmap_points_.vertex_min[i]);
    dist_to_map_edge[i][1] =
        fabs(lidar_xyz(i) - localmap_points_.vertex_max[i]);

    if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * config_.det_range ||
        dist_to_map_edge[i][1] <= MOV_THRESHOLD * config_.det_range) {
      need_move = true;
    }
  }

  if (!need_move) return;

  BoxPointType new_localmap_points, tmp_boxpoints;
  new_localmap_points = localmap_points_;
  float mov_dist = std::max((config_.cube_len - 2.0 * MOV_THRESHOLD * config_.det_range) * 0.5 * 0.9,
                            static_cast<double>(config_.det_range * (MOV_THRESHOLD - 1)));

  for (int i = 0; i < 3; i++) {
    tmp_boxpoints = localmap_points_;
    if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * config_.det_range) {
      new_localmap_points.vertex_max[i] -= mov_dist;
      new_localmap_points.vertex_min[i] -= mov_dist;
      tmp_boxpoints.vertex_min[i] = localmap_points_.vertex_max[i] - mov_dist;
      cub_needrm_.push_back(tmp_boxpoints);
    } else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * config_.det_range) {
      new_localmap_points.vertex_max[i] += mov_dist;
      new_localmap_points.vertex_min[i] += mov_dist;
      tmp_boxpoints.vertex_max[i] = localmap_points_.vertex_min[i] + mov_dist;
      cub_needrm_.push_back(tmp_boxpoints);
    }
  }

  localmap_points_ = new_localmap_points;

  if (!cub_needrm_.empty()) {
    ikd_tree_.Delete_Point_Boxes(cub_needrm_);
  }
}

void FastLIO2Core::mapIncremental() {
  PointVector PointToAdd;
  PointVector PointNoNeedDownsample;
  PointToAdd.reserve(feats_down_size_);
  PointNoNeedDownsample.reserve(feats_down_size_);

  for (int i = 0; i < feats_down_size_; i++) {
    // transform to world frame
    pointBodyToWorld(
        &(feats_down_body_->points[i]), &(feats_down_world_->points[i]), latest_state_);

    // decide if we need to add to map
    if (!nearest_points_[i].empty() && flg_EKF_inited_) {
      const PointVector &points_near = nearest_points_[i];
      bool need_add                  = true;

      PointType mid_point;
      mid_point.x =
          std::floor(feats_down_world_->points[i].x / config_.filter_size_map_min) *
              config_.filter_size_map_min +
          0.5 * config_.filter_size_map_min;
      mid_point.y =
          std::floor(feats_down_world_->points[i].y / config_.filter_size_map_min) *
              config_.filter_size_map_min +
          0.5 * config_.filter_size_map_min;
      mid_point.z =
          std::floor(feats_down_world_->points[i].z / config_.filter_size_map_min) *
              config_.filter_size_map_min +
          0.5 * config_.filter_size_map_min;

      float dist = calc_dist(feats_down_world_->points[i], mid_point);
      if (std::fabs(points_near[0].x - mid_point.x) > 0.5f * config_.filter_size_map_min &&
          std::fabs(points_near[0].y - mid_point.y) > 0.5f * config_.filter_size_map_min &&
          std::fabs(points_near[0].z - mid_point.z) > 0.5f * config_.filter_size_map_min) {
        PointNoNeedDownsample.push_back(feats_down_world_->points[i]);
        continue;
      }

      for (int readd_i = 0; readd_i < NUM_MATCH_POINTS; readd_i++) {
        if (points_near.size() < NUM_MATCH_POINTS) break;
        if (calc_dist(points_near[readd_i], mid_point) < dist) {
          need_add = false;
          break;
        }
      }
      if (need_add) {
        PointToAdd.push_back(feats_down_world_->points[i]);
      }
    } else {
      // no nearest points, or not in EKF inited
      PointToAdd.push_back(feats_down_world_->points[i]);
    }
  }

  double st_time  = omp_get_wtime();
  add_point_size_ = ikd_tree_.Add_Points(PointToAdd, true);
  ikd_tree_.Add_Points(PointNoNeedDownsample, false);

  add_point_size_          = PointToAdd.size() + PointNoNeedDownsample.size();
  kdtree_incremental_time_ = omp_get_wtime() - st_time;
}

void FastLIO2Core::integrateIMU(esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state,
                                const std::deque<sensor_msgs::msg::Imu> &imu_queue) {
  imu_processor_->IntegrateIMU(imu_queue, kf_state);
}

}  // namespace spark_fast_lio
