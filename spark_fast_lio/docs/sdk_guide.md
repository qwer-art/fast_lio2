# SparkLioSdk 离线数据同步使用指南

## 概述

`SparkLioSdk` 是一个脱离 ROS2 节点的离线 SDK，直接从 rosbag2 文件中读取 LiDAR + IMU 数据，并通过 `syncPackages` 算法按时间戳同步后逐帧返回。适用于离线算法开发、数据回放调试等场景。

## 编译

```bash
cd /home/jerett/OpenProject/LidarSlam/spark-fast-lio
source /opt/ros/humble/setup.bash
colcon build --packages-select spark_fast_lio
source install/setup.bash
```

编译产物：

| 产物 | 类型 | 说明 |
|------|------|------|
| `libspark_lio_sdk.so` | 共享库 | SDK 库，可被外部程序链接 |
| `spark_lio_sdk_demo` | 可执行文件 | 命令行 demo，遍历 bag 输出同步帧信息 |

## 命令行运行

```bash
ros2 run spark_fast_lio spark_lio_sdk_demo <bag_path> [lidar_topic] [imu_topic]
```

参数说明：

| 参数 | 必选 | 默认值 | 说明 |
|------|------|--------|------|
| `bag_path` | 是 | - | rosbag2 目录路径（包含 `metadata.yaml`） |
| `lidar_topic` | 否 | `/velodyne_points` | PointCloud2 话题名 |
| `imu_topic` | 否 | `/imu/data` | IMU 话题名 |

示例：

```bash
ros2 run spark_fast_lio spark_lio_sdk_demo \
  /home/jerett/OpenProject/LidarSlam/spark-fast-lio/spark_fast_lio/data/10_14_hathor/asset_data \
  /acl_jackal2/lidar_points \
  /acl_jackal2/forward/imu
```

输出示例：

```
[SparkLioSdk] Bag opened: /data/10_14_hathor
[SparkLioSdk]   Message count: 123456
[SparkLioSdk]   Topics:
[SparkLioSdk]     /acl_jackal2/lidar_points (sensor_msgs/msg/PointCloud2, 6000 msgs)
[SparkLioSdk]     /acl_jackal2/forward/imu (sensor_msgs/msg/Imu, 117456 msgs)
[Frame 1] lidar_pts=28756  imu_count=20  t=[1697280000.000000, 1697280000.100000]
[Frame 2] lidar_pts=29102  imu_count=20  t=[1697280000.100000, 1697280000.200000]
...
Total synchronized frames: 6000
```

## C++ API 使用

### 基本用法

```cpp
#include "spark_lio_sdk.h"

int main() {
  // 1. 配置
  spark_fast_lio::SparkLioSdk::Config config;
  config.lidar_type       = VELO16;  // AVIA=1, VELO16=2, OUST64=3, KMOUST64=4
  config.scan_line        = 16;
  config.scan_rate        = 10;
  config.time_unit        = US;      // SEC=0, MS=1, US=2, NS=3
  config.blind            = 0.01;
  config.point_filter_num = 1;
  config.verbose          = true;

  // 2. 打开 bag
  spark_fast_lio::SparkLioSdk sdk;
  if (!sdk.open("/path/to/bag", "/lidar_topic", "/imu_topic", config)) {
    return 1;
  }

  // 3. 逐帧获取同步数据
  spark_fast_lio::MeasureGroup meas;
  while (sdk.getSyncedData(meas)) {
    // meas.lidar          : PointCloudXYZI::Ptr  — 预处理后的点云
    // meas.lidar_beg_time : double               — 扫描起始时间 (秒)
    // meas.lidar_end_time : double               — 扫描结束时间 (秒)
    // meas.imu            : deque<Imu::ConstSharedPtr> — 同步的 IMU 测量

    // 你的算法写在这里
  }

  // 4. 关闭
  sdk.close();
  return 0;
}
```

### Config 参数说明

| 字段 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `lidar_type` | int | `AVIA` (1) | LiDAR 型号：`AVIA`=1, `VELO16`=2, `OUST64`=3, `KMOUST64`=4 |
| `scan_line` | int | 16 | 扫描线数 |
| `scan_rate` | int | 10 | 扫描频率 (Hz)，仅 Velodyne 需要设置 |
| `time_unit` | int | `US` (2) | 点云中时间戳单位：`SEC`=0, `MS`=1, `US`=2, `NS`=3 |
| `blind` | double | 0.01 | 近距离盲区 (m)，小于此距离的点被剔除 |
| `point_filter_num` | int | 1 | 预处理点滤波间隔（1=不滤波） |
| `verbose` | bool | false | 是否输出调试信息 |

### MeasureGroup 数据结构

```cpp
struct MeasureGroup {
  double lidar_beg_time;                    // 扫描起始时间 (秒)
  double lidar_end_time;                    // 扫描结束时间 (秒)
  PointCloudXYZI::Ptr lidar;                // 预处理后的点云 (pcl::PointXYZINormal)
  std::deque<std::shared_ptr<const sensor_msgs::msg::Imu>> imu;  // 同步的 IMU 数据

  V3D getMeanAcc();  // 获取 IMU 平均加速度
};
```

## 内部工作流程

```
┌─────────────────────────────────────────────────────┐
│                   SparkLioSdk                        │
│                                                      │
│  rosbag2_cpp::Reader                                 │
│    │                                                 │
│    ├── read_next() ──► PointCloud2                   │
│    │     └── Preprocess::process() ──► lidar_buffer_ │
│    │                                                 │
│    └── read_next() ──► Imu ──► imu_buffer_           │
│                                                      │
│  syncPackages()                                      │
│    ├── 取 lidar_buffer_.front() 作为当前帧           │
│    ├── 计算 lidar_end_time_ (扫描结束时间)           │
│    ├── 等待 imu_buffer_ 中 IMU 时间 >= lidar_end_time│
│    └── 取出 [lidar_beg_time, lidar_end_time] 内的 IMU│
│                                                      │
│  getSyncedData()                                     │
│    └── 循环读取 bag → 填充 buffer → syncPackages     │
│        直到同步成功或 bag 读完                        │
└─────────────────────────────────────────────────────┘
```

### syncPackages 同步算法

与 `SPARKFastLIO2::syncPackages()` 逻辑一致：

1. 从 `lidar_buffer_` 取出最早一帧点云
2. 根据点云最后一个点的 `curvature`（相对时间偏移）计算 `lidar_end_time_`
3. 等待 `imu_buffer_` 中最新 IMU 时间戳 >= `lidar_end_time_`
4. 取出时间在 `[lidar_beg_time, lidar_end_time]` 范围内的所有 IMU 数据
5. 弹出已消费的 lidar 帧，返回同步后的 `MeasureGroup`

## 文件结构

```
spark_fast_lio/
├── include/
│   └── spark_lio_sdk.h          # SDK 头文件
├── src/
│   ├── spark_lio_sdk.cpp        # SDK 实现 (rosbag2 读取 + syncPackages)
│   └── main.cpp                 # SDK demo 可执行程序
└── CMakeLists.txt               # 包含 spark_lio_sdk 库和 spark_lio_sdk_demo 目标
```

## 与在线模式的对比

| 特性 | 在线模式 (`SPARKFastLIO2`) | 离线 SDK (`SparkLioSdk`) |
|------|---------------------------|--------------------------|
| 数据来源 | ROS2 订阅实时话题 | rosbag2_cpp 离线读取 |
| 需要 ROS2 节点 | 是 | 否 |
| 时间同步 | 回调 + 定时器驱动 | 按需读取 + 同步 |
| 调用方式 | `ros2 launch` | `getSyncedData()` 迭代 |
| 适用场景 | 实时运行 | 离线开发/调试/回放 |
