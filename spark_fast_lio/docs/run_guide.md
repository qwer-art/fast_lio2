# SPARK-FAST-LIO 运行说明

## 数据集下载

从 [Dropbox](https://www.dropbox.com/scl/fo/i56kucdzxpzq1mr5jula7/ALJpdqvOZT1hTaQXEePCvyI?rlkey=y5bvslyazf09erko7gl0aylll&e=1&dl=0) 下载以下推荐数据集：

| 数据集 | 说明 | 推荐度 |
|--------|------|--------|
| `10_14_acl_jackal` | MIT Campus (Kimera-Multi) | ⭐⭐⭐ 推荐 |
| `colosseo_train0` | VBR Colosseum | ⭐⭐⭐ 推荐 |
| `10_14_hathor` | MIT Campus (Kimera-Multi) | ⚠️ 数据格式可能有兼容性问题 |

下载后放置到：
```
/home/jerett/OpenProject/LidarSlam/spark-fast-lio/spark_fast_lio/data/
```

## 数据集路径

| 数据集 | 路径 |
|--------|------|
| 10_14_acl_jackal | `/home/jerett/OpenProject/LidarSlam/spark-fast-lio/spark_fast_lio/data/10_14_acl_jackal` |
| 10_14_hathor | `/home/jerett/OpenProject/LidarSlam/spark-fast-lio/spark_fast_lio/data/10_14_hathor` |
| colosseo_train0 | `/home/jerett/OpenProject/LidarSlam/spark-fast-lio/spark_fast_lio/data/colosseo_train0` |

## 运行步骤

### 1. 初始化环境

```bash
source /home/jerett/OpenProject/LidarSlam/spark-fast-lio/spark_fast_lio/init_env.sh
```

如果是首次运行或代码有更新，需要先编译：

```bash
source /home/jerett/OpenProject/LidarSlam/spark-fast-lio/spark_fast_lio/init_env.sh build
```

### 2. 启动节点

```bash
ros2 launch spark_fast_lio mapping_mit_campus.launch.yaml scene_id:=hathor robot_name:=acl_jackal2
```

> **注意**：`scene_id` 对应 bag 的话题前缀，`robot_name` 对应数据采集的机器人名称。

### 3. 播放 bag

在新终端中执行：

```bash
source /home/jerett/OpenProject/LidarSlam/spark-fast-lio/spark_fast_lio/init_env.sh
ros2 bag play /home/jerett/OpenProject/LidarSlam/spark-fast-lio/spark_fast_lio/data/10_14_hathor
```

## 文件路径

| 文件 | 路径 |
|------|------|
| 环境初始化脚本 | `/home/jerett/OpenProject/LidarSlam/spark-fast-lio/spark_fast_lio/init_env.sh` |
| Launch 文件 | `/home/jerett/OpenProject/LidarSlam/spark-fast-lio/spark_fast_lio/launch/mapping_mit_campus.launch.yaml` |
| 配置文件 | `/home/jerett/OpenProject/LidarSlam/spark-fast-lio/spark_fast_lio/config/velodyne_mit.yaml` |
| Bag 文件 | `/home/jerett/OpenProject/LidarSlam/spark-fast-lio/spark_fast_lio/data/10_14_hathor/10_14_hathor.db3` |

## hathor 数据集参数

| 参数 | 值 | 说明 |
|------|-----|------|
| lidar_type | 2 | Velodyne LiDAR |
| scan_line | 16 | 16线激光雷达 |
| scan_rate | 10 | 10 Hz |
| timestamp_unit | 2 | microsecond |
