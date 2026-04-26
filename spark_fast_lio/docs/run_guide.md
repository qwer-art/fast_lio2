# SPARK-FAST-LIO 运行说明

## 1. 运行LIO系统

```bash
source /home/jerett/OpenProject/LidarSlam/spark-fast-lio/spark_fast_lio/init_env.sh
ros2 launch spark_fast_lio mapping_mit_campus.launch.yaml scene_id:=hathor robot_name:=acl_jackal2
```

## 2. 播放Bag

```bash
source /home/jerett/OpenProject/LidarSlam/spark-fast-lio/spark_fast_lio/init_env.sh
ros2 bag play /home/jerett/OpenProject/LidarSlam/spark-fast-lio/spark_fast_lio/data/10_14_hathor

## ros2 bag play /home/jerett/OpenProject/LidarSlam/spark-fast-lio/spark_fast_lio/data/10_14_hathor --clock --rate 5.0
```

## 3. ROS2可视化

```bash
ros2 run plotjuggler plotjuggler
```

**Streaming** → **Start: ROS2 Topic Subscriber** → 选择 `/odometry`

## 4. 离线调试数据记录

```bash
source /home/jerett/OpenProject/LidarSlam/spark-fast-lio/spark_fast_lio/init_env.sh
python3 /home/jerett/OpenProject/LidarSlam/spark-fast-lio/spark_fast_lio/scripts/lio_debug_recorder.py
```

运行结束后，数据保存在：
```
logs/<timestamp>/debug_data/*.csv
```

PlotJuggler加载：**File** → **Load Data** → 选择CSV文件

---

## 数据集路径

| 数据集 | 路径 |
|--------|------|
| 10_14_hathor | `spark_fast_lio/data/10_14_hathor` |
| 10_14_acl_jackal | `spark_fast_lio/data/10_14_acl_jackal` |
| colosseo_train0 | `spark_fast_lio/data/colosseo_train0` |

## 调试数据说明

| 文件 | 内容 |
|------|------|
| `lio_debug_pose.csv` | 最终Pose + IMU预积分Pose |
| `lio_debug_delta.csv` | 帧间Delta Pose |
| `lio_debug_bias.csv` | IMU Bias |
| `lio_debug_quality.csv` | 匹配质量 |
| `lio_debug_velocity.csv` | 速度 |
