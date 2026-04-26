# Parser SDK 使用指南

## 1. 编译

```bash
# 进入项目根目录
cd /home/jerett/OpenProject/LidarSlam/spark-fast-lio

# 编译
source spark_fast_lio/init_env.sh build
```

编译产物：
- `spark_lio_sdk_demo` - 数据解析工具

## 2. 运行

```bash
ros2 run spark_fast_lio spark_lio_sdk_demo <bag_path> <asset_path>
```

**参数说明：**
- `bag_path`：rosbag2 目录路径（包含 `metadata.yaml`）
- `asset_path`：输出文件保存路径

**硬编码话题：**
- LiDAR: `/hathor/lidar_points`
- IMU: `/hathor/forward/imu`

**示例：**
```bash
ros2 run spark_fast_lio spark_lio_sdk_demo \
  /home/jerett/OpenProject/LidarSlam/spark-fast-lio/spark_fast_lio/data/10_14_hathor \
  /home/jerett/OpenProject/LidarSlam/spark-fast-lio/spark_fast_lio/data/output
```

## 3. 输出产物

```
<asset_path>/
├── frame_info/    # 帧信息 JSON
├── lidar/         # 点云 PLY 文件
└── imu/           # IMU 数据 CSV
```

### 文件格式

**frame_info JSON：**
```json
{
  "frame_time": "001666028921_450000000000",
  "begin_time": 1666028921.350000000,
  "end_time": 1666028921.450000000
}
```

**lidar PLY：** 标准 PLY 格式，包含 x, y, z, intensity

**imu CSV：**
```csv
timestamp,ax,ay,az,gx,gy,gz,qx,qy,qz,qw
```

## 4. 注意事项

- 话题名硬编码，不支持运行时修改
- 时间戳格式：12位秒数_12位皮秒数（100ms对齐）

