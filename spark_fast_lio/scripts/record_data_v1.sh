#!/bin/bash

# 获取脚本所在目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 数据保存目录
DATA_DIR="${SCRIPT_DIR}/data"

# 创建data目录（如果不存在）
mkdir -p ${DATA_DIR}

# 文件名：data/lio_{录制开始时间}
OUTPUT_NAME="${DATA_DIR}/lio_$(date +%Y%m%d_%H%M%S)"

# 录制话题
ros2 bag record -o ${OUTPUT_NAME} \
  /odometry \
  /path \
  /tf \
  /tf_static \
  /debug/delta_pose \
  /debug/imu_bias_acc \
  /debug/imu_bias_gyro \
  /debug/imu_preint_pose \
  /debug/match_quality \
  /debug/velocity

# /hathor/forward/imu \
# /hathor/lidar_points \