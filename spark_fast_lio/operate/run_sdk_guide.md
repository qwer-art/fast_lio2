# Run SDK 使用指南

## 1. 编译

```bash
# 进入项目根目录
cd /home/jerett/OpenProject/LidarSlam/spark-fast-lio

# 编译
source spark_fast_lio/init_env.sh build
```

编译产物：
- `spark_lio_run_sdk` - Fast-LIO2 算法运行工具

## 2. 运行

```bash
ros2 run spark_fast_lio spark_lio_run_sdk <bag_path> <asset_path> <param_path>
```

**参数说明：**
- `bag_path`：rosbag2 目录路径（包含 `metadata.yaml`）
- `asset_path`：输出文件保存路径
- `param_path`：YAML配置文件路径

**硬编码话题：**
- LiDAR: `/hathor/lidar_points`
- IMU: `/hathor/forward/imu`

**示例：**
```bash
ros2 run spark_fast_lio spark_lio_run_sdk \
  /home/jerett/OpenProject/LidarSlam/spark-fast-lio/spark_fast_lio/data/10_14_hathor \
  /home/jerett/OpenProject/LidarSlam/spark-fast-lio/spark_fast_lio/data/ekf \
  /home/jerett/OpenProject/LidarSlam/spark-fast-lio/spark_fast_lio/config/kimera_multi/hathor.yaml
```

## 3. 配置文件

配置文件使用YAML格式，与ROS节点使用的配置文件相同。

**配置文件位置：**
```
spark_fast_lio/config/
├── kimera_multi/
│   ├── hathor.yaml
│   ├── thoth.yaml
│   └── ...
├── velodyne.yaml
└── ...
```

**配置参数说明：**

| 参数 | 说明 | 默认值 |
|------|------|--------|
| **预处理参数** | | |
| `preprocess.lidar_type` | LiDAR类型 (1=AVIA, 2=VELO16, 3=OUST64) | 2 |
| `preprocess.scan_line` | 扫描线数 | 16 |
| `preprocess.scan_rate` | 扫描频率 (Hz) | 10 |
| `preprocess.timestamp_unit` | 时间戳单位 (0=秒, 1=毫秒, 2=微秒, 3=纳秒) | 2 |
| `preprocess.blind` | 近点过滤距离 (m) | 0.01 |
| `preprocess.blind_for_human_pilot` | 飞手区域过滤距离 (m) | 1.5 |
| **滤波参数** | | |
| `mapping.acc_cov` | 加速度计协方差 | 0.1 |
| `mapping.gyr_cov` | 陀螺仪协方差 | 0.1 |
| `mapping.b_acc_cov` | 加速度计偏置协方差 | 0.0001 |
| `mapping.b_gyr_cov` | 陀螺仪偏置协方差 | 0.0001 |
| `mapping.det_range` | 检测范围 (m) | 300.0 |
| `mapping.extrinsic_est_en` | 是否在线估计外参 | false |
| `mapping.extrinsic_T` | LiDAR到IMU的平移 [x, y, z] | [0, 0, 0] |
| `mapping.extrinsic_R` | LiDAR到IMU的旋转 (行优先, 9个值) | 单位阵 |
| **其他参数** | | |
| `filter_size_map` | 地图滤波尺寸 | 0.5 |
| `point_filter_num` | 点云滤波间隔 | 4 |
| `max_iteration` | 最大迭代次数 | 4 |
| `cube_side_length` | 局部地图边长 | 200.0 |
| `verbose` | 详细输出 | false |

## 4. 输出产物

```
<asset_path>/
├── pred_state/     # 预测状态 CSV
│   ├── header.txt  # CSV列说明
│   ├── 001666028921_500000000000.csv
│   └── ...
├── pred_cov/       # 预测协方差 CSV (23×23矩阵)
├── update_state/   # 更新状态 CSV
└── update_cov/     # 更新协方差 CSV (23×23矩阵)
```

### 状态CSV格式

```
frame_time,pos_x,pos_y,pos_z,quat_w,quat_x,quat_y,quat_z,
euler_roll,euler_pitch,euler_yaw,vel_x,vel_y,vel_z,
bg_x,bg_y,bg_z,ba_x,ba_y,ba_z,grav_x,grav_y,grav_z,
Ril_w,Ril_x,Ril_y,Ril_z,Til_x,Til_y,Til_z
```

### 预测 vs 更新

- **pred_state/pred_cov**：IMU预积分后的状态（ESKF更新前）
- **update_state/update_cov**：ESKF测量更新后的状态

## 5. 算法流程

```
MeasureGroup (LiDAR + IMU)
    ↓
IMU预积分
    ↓
预测状态 & 协方差
    ↓
点云降采样 & 特征提取
    ↓
ikd-tree地图匹配
    ↓
ESKF测量更新
    ↓
更新状态 & 协方差
    ↓
地图增量更新
```

## 6. 注意事项

- 话题名硬编码，不支持运行时修改
- 第一帧用于初始化，不输出状态
- 时间戳格式：12位秒数_12位皮秒数（100ms对齐）
- 协方差矩阵23维：位置(3) + 姿态(3) + 外参(6) + 速度(3) + 偏置(6) + 重力(2)
