# LIO 系统调试方案

## 监控目标

当位置出现异常时，通过PlotJuggler实时查看各环节输出，快速定位问题来源。

## 数据流与监控点

```
原始IMU ──► IMU预积分 ──► 帧间Delta ──► EKF更新 ──► 最终Pose
   │            │            │            │            │
   ▼            ▼            ▼            ▼            ▼
 /imu_raw   /debug/imu_preint_pose   /debug/delta_pose  /odometry
```

## 调试Topic列表

| Topic | 类型 | 内容 |
|-------|------|------|
| `/odometry` | `Odometry` | EKF优化后的位姿 |
| `/debug/imu_preint_pose` | `PoseStamped` | IMU预积分位姿 |
| `/debug/delta_pose` | `Pose` | 帧间相对运动 |
| `/debug/imu_bias_gyro` | `Vector3Stamped` | 陀螺仪偏置 bg |
| `/debug/imu_bias_acc` | `Vector3Stamped` | 加速度计偏置 ba |
| `/debug/match_quality` | `Float64MultiArray` | [特征点数, 残差, 求解时间] |
| `/debug/velocity` | `Vector3Stamped` | EKF估计速度 |

## 异常诊断流程

```
位置异常
    │
    ├─► Delta Pose异常？ ──是──► 单帧运动估计问题
    │         │                    ├─► 检查IMU数据质量
    │         否                   └─► 检查预积分参数
    │         │
    │         ▼
    ├─► IMU预积分与最终Pose差异大？ ──是──► 激光校正问题
    │         │                         ├─► 检查特征点数量
    │         否                        └─► 检查ICP残差
    │         │
    │         ▼
    ├─► Bias发散？ ──是──► IMU初始化问题或长时间静止
    │         │
    │         否
    │         │
    │         ▼
    └─► 匹配质量差？ ──是──► 点云匹配失败
              │              ├─► 特征点过少
              否             └─► 残差过大
              │
              ▼
         检查EKF参数或协方差设置
```

## 数据记录与分析

### 1. 启动LIO节点
```bash
source /home/jerett/OpenProject/LidarSlam/spark-fast-lio/spark_fast_lio/init_env.sh
ros2 launch spark_fast_lio mapping_mit_campus.launch.yaml scene_id:=hathor robot_name:=acl_jackal2
```

### 2. 启动数据记录脚本
```bash
source /home/jerett/OpenProject/LidarSlam/spark-fast-lio/spark_fast_lio/init_env.sh
python3 /home/jerett/OpenProject/LidarSlam/spark-fast-lio/spark_fast_lio/scripts/lio_debug_recorder.py
```

### 3. 播放bag或实时运行
```bash
ros2 bag play /home/jerett/OpenProject/LidarSlam/spark-fast-lio/spark_fast_lio/data/10_14_hathor
```

### 4. PlotJuggler分析CSV
```bash
ros2 run plotjuggler plotjuggler
```
**File** → **Load Data** → 选择 `logs/<timestamp>/debug_data/*.csv`

## 输出文件

| 文件 | 内容 |
|------|------|
| `lio_debug_pose.csv` | 最终Pose + IMU预积分Pose |
| `lio_debug_delta.csv` | 帧间Delta Pose |
| `lio_debug_bias.csv` | IMU Bias (bg, ba) |
| `lio_debug_quality.csv` | 匹配质量 (特征点数, 残差, 求解时间) |
| `lio_debug_velocity.csv` | EKF估计速度 |
