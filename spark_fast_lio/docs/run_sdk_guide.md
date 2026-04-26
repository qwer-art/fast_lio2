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
ros2 run spark_fast_lio spark_lio_run_sdk <bag_path> <asset_path>
```

**参数说明：**
- `bag_path`：rosbag2 目录路径（包含 `metadata.yaml`）
- `asset_path`：输出文件保存路径

**硬编码话题：**
- LiDAR: `/hathor/lidar_points`
- IMU: `/hathor/forward/imu`

**示例：**
```bash
ros2 run spark_fast_lio spark_lio_run_sdk \
  /home/jerett/OpenProject/LidarSlam/spark-fast-lio/spark_fast_lio/data/10_14_hathor \
  /home/jerett/OpenProject/LidarSlam/spark-fast-lio/spark_fast_lio/data/output
```

## 3. 输出产物

```
<asset_path>/
├── pred_state/     # 预测状态 JSON
├── pred_cov/       # 预测协方差 CSV
├── update_state/   # 更新状态 JSON
└── update_cov/     # 更新协方差 CSV
```

### 文件格式

**状态 JSON：**
```json
{
  "frame_time": "001666028921_550000000000",
  "position": [1.234, 2.345, 3.456],
  "orientation_quat": [0.999, 0.001, 0.002, 0.003],
  "orientation_euler_deg": [0.12, 0.23, 0.34],
  "velocity": [0.12, 0.23, 0.34],
  "bias_gyro": [0.000001, 0.000002, 0.000003],
  "bias_acc": [0.000012, 0.000023, 0.000034],
  "gravity": [0.0, 0.0, -9.81],
  "offset_R_L_I": [1.0, 0.0, 0.0, 0.0],
  "offset_T_L_I": [0.0, 0.0, 0.0]
}
```

**协方差 CSV：** 23×23 矩阵，科学计数法格式

### 预测 vs 更新

- **pred_state/pred_cov**：IMU预积分后的状态（ESKF更新前）
- **update_state/update_cov**：ESKF测量更新后的状态

## 4. 算法流程

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

## 5. 注意事项

- 话题名硬编码，不支持运行时修改
- 第一帧用于初始化，不输出状态
- 时间戳格式：12位秒数_12位皮秒数（100ms对齐）
- 协方差矩阵23维：位置(3) + 姿态(3) + 外参(6) + 速度(3) + 偏置(6) + 重力(2)
