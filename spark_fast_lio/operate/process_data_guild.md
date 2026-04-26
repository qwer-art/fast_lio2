# Output数据解析指南

## 1. 概述

`parser_output` 工具用于将SLAM运行产生的EKF数据整理成CSV文件，便于使用PlotJuggler进行离线分析。

## 2. 使用方法

### 2.1 Shell脚本调用（推荐）

```bash
./parser_output.sh
```

修改 `parser_output.sh` 开头的路径变量即可：

```bash
asset_data=/home/jerett/OpenProject/LidarSlam/spark-fast-lio/spark_fast_lio/data/asset_data
ekf=/home/jerett/OpenProject/LidarSlam/spark-fast-lio/spark_fast_lio/data/ekf
```

CSV输出路径自动生成：`{ekf}/analyse_{时间}.csv`

### 2.2 Python脚本直接调用

```bash
python3 parser_output.py <asset_data> <ekf>
```

**参数说明：**
- `asset_data`: 数据同步后的文件夹地址 (包含frame_info, imu等子目录)
- `ekf`: 滤波后的文件夹地址 (包含pred_state, update_state等子目录)

**示例：**
```bash
python3 parser_output.py \
    /home/jerett/OpenProject/LidarSlam/spark-fast-lio/spark_fast_lio/data/asset_data \
    /home/jerett/OpenProject/LidarSlam/spark-fast-lio/spark_fast_lio/data/ekf
```

**输出：**
- CSV文件保存至 `{ekf}/analyse_{时间}.csv`

## 3. 输入数据结构

**asset_data目录：**
```
<asset_data>/
├── frame_info/     # 帧时间信息
└── imu/            # IMU数据
```

**ekf目录：**
```
<ekf>/
├── pred_state/     # 预测状态 CSV文件 (必需)
├── update_state/   # 更新状态 CSV文件 (必需)
├── pred_cov/       # 预测协方差矩阵 (可选)
└── update_cov/     # 更新协方差矩阵 (可选)
```

## 4. 输出CSV格式

| 列名 | 说明 | 单位 |
|------|------|------|
| `frame_time` | 帧时间戳字符串 | - |
| `frame_time_sec` | 帧时间戳 (秒) | s |
| `lidar_begin` | LiDAR扫描开始时间 | s |
| `lidar_end` | LiDAR扫描结束时间 | s |
| `imu_begin` | IMU数据开始时间 | s |
| `imu_end` | IMU数据结束时间 | s |
| `pred_x/y/z` | 预测位置 | m |
| `pred_roll/pitch/yaw` | 预测姿态 (欧拉角) | rad |
| `pred_bg_x/y/z` | 预测陀螺仪bias | rad/s |
| `pred_ba_x/y/z` | 预测加速度计bias | m/s² |
| `pred_Ril_w/x/y/z` | 预测LiDAR-IMU外参旋转 (四元数) | - |
| `pred_Til_x/y/z` | 预测LiDAR-IMU外参平移 | m |
| `update_x/y/z` | 更新位置 | m |
| `update_roll/pitch/yaw` | 更新姿态 (欧拉角) | rad |
| `update_bg_x/y/z` | 更新陀螺仪bias | rad/s |
| `update_ba_x/y/z` | 更新加速度计bias | m/s² |
| `update_Ril_w/x/y/z` | 更新LiDAR-IMU外参旋转 (四元数) | - |
| `update_Til_x/y/z` | 更新LiDAR-IMU外参平移 | m |

**说明：**
- 四元数自动转换为欧拉角 (roll, pitch, yaw)
- frame_time格式: `SSSSSSSSSSSS_PPPPPPPPPPPP` (12位秒数_12位皮秒数)

## 5. PlotJuggler可视化

```bash
ros2 run plotjuggler plotjuggler
```

1. 打开PlotJuggler
2. `File` -> `Load Data` -> 选择生成的CSV文件
3. 在左侧列表中选择要绘制的曲线

## 6. 文件位置

| 文件 | 路径 |
|------|------|
| Python脚本 | `spark_fast_lio/scripts/parser_output.py` |
| Shell脚本 | `spark_fast_lio/scripts/parser_output.sh` |
| 使用指南 | `spark_fast_lio/operate/process_data_guild.md` |
