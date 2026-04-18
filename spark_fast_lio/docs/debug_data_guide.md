# LIO Debug Data 分析指南

## 数据文件说明

| 文件 | 内容 | 用途 |
|------|------|------|
| `lio_debug_quality.csv` | 特征数、残差、求解时间 | 匹配质量分析 |
| `lio_debug_bias.csv` | 陀螺仪/加速度计Bias | IMU状态收敛分析 |
| `lio_debug_velocity.csv` | 三轴速度 | 运动估计分析 |
| `lio_debug_delta.csv` | 帧间Delta Pose | 运动增量分析 |
| `lio_debug_pose.csv` | 最终Pose vs IMU预积分Pose | 轨迹对比分析 |

## 字段说明

### 通用字段
- `timestamp`: 原始ROS时间戳（秒）
- `frame_time`: 对齐到50ms网格的时间戳，用于PlotJuggler多数据对齐显示

### lio_debug_quality.csv
```
timestamp, frame_time, feat_num, res_mean, solve_time
```
- `feat_num`: 当前帧使用的特征点数量
- `res_mean`: 匹配残差均值（越小越好）
- `solve_time`: 求解耗时（秒）

### lio_debug_bias.csv
```
timestamp, frame_time, bg_x, bg_y, bg_z, ba_x, ba_y, ba_z
```
- `bg_x/y/z`: 陀螺仪Bias (rad/s)
- `ba_x/y/z`: 加速度计Bias (m/s²)

### lio_debug_velocity.csv
```
timestamp, frame_time, vx, vy, vz
```
- `vx/vy/vz`: 三轴速度 (m/s)

### lio_debug_delta.csv
```
timestamp, frame_time, delta_x, delta_y, delta_z, delta_roll, delta_pitch, delta_yaw
```
- `delta_x/y/z`: 帧间位置增量 (m)
- `delta_roll/pitch/yaw`: 帧间姿态增量 (rad)

### lio_debug_pose.csv
```
timestamp, frame_time,
odom_x, odom_y, odom_z, odom_qx, odom_qy, odom_qz, odom_qw, odom_roll, odom_pitch, odom_yaw,
preint_x, preint_y, preint_z, preint_qx, preint_qy, preint_qz, preint_qw, preint_roll, preint_pitch, preint_yaw
```
- `odom_*`: 最终优化后的Pose
- `preint_*`: IMU预积分预测的Pose（用于对比分析漂移）

---

## Debug分析流程

### 第一步：检查匹配质量 (L4)

**文件**: `lio_debug_quality.csv`

**正常指标**:
- `feat_num` > 500（特征点足够）
- `res_mean` < 0.1（残差较小）
- `solve_time` < 0.02s（求解快速）

**异常现象**:
| 现象 | 可能原因 |
|------|----------|
| feat_num 剧烈波动或过低 | 点云质量差、特征提取参数不当 |
| res_mean 持续增大 | 匹配退化、环境特征单一 |
| solve_time 过大 | 系统负载高、优化问题病态 |

---

### 第二步：检查IMU Bias收敛 (L3)

**文件**: `lio_debug_bias.csv`

**正常指标**:
- Bias应快速收敛到稳定值
- 收敛后波动幅度小

**异常现象**:
| 现象 | 可能原因 |
|------|----------|
| Bias持续发散 | IMU数据异常、初始化失败 |
| Bias剧烈波动 | 运动过于剧烈、观测约束不足 |
| Bias收敛过慢 | 初始化参数不当 |

---

### 第三步：检查运动估计 (L2)

**文件**: `lio_debug_velocity.csv`, `lio_debug_delta.csv`

**正常指标**:
- 速度变化平滑
- Delta Pose与实际运动一致

**异常现象**:
| 现象 | 可能原因 |
|------|----------|
| 速度突变 | IMU/Lidar数据不同步 |
| Delta异常大 | 帧间匹配失败 |
| 速度持续非零（静止时） | 零速修正失效 |

---

### 第四步：检查轨迹对比 (L1)

**文件**: `lio_debug_pose.csv`

**分析方法**:
- 对比 `odom_*` 和 `preint_*` 曲线
- 两者应高度一致，差异反映IMU积分漂移

**正常指标**:
- 两条曲线基本重合
- 差异在厘米级以内

**异常现象**:
| 现象 | 可能原因 |
|------|----------|
| 曲线分离 | IMU预积分与Lidar匹配不一致 |
| preint超前/滞后 | 时间同步问题 |
| 差异持续增大 | IMU参数标定误差 |

---

## PlotJuggler使用方法

### 加载数据
1. 打开 PlotJuggler
2. File → Load Data → 选择多个CSV文件（Ctrl多选）
3. 所有数据共享 `frame_time` 时间轴

### 推荐视图布局

**视图1 - 匹配质量**:
- `feat_num` (左Y轴)
- `res_mean` (右Y轴)

**视图2 - IMU Bias**:
- `bg_x`, `bg_y`, `bg_z`
- `ba_x`, `ba_y`, `ba_z`

**视图3 - 速度**:
- `vx`, `vy`, `vz`

**视图4 - 轨迹对比**:
- `odom_x` vs `preint_x`
- `odom_y` vs `preint_y`
- `odom_z` vs `preint_z`

---

## 快速诊断checklist

```
□ feat_num 稳定且足够 (>500)
□ res_mean 收敛到小值 (<0.1)
□ Bias 收敛且稳定
□ 速度曲线平滑无突变
□ odom与preint曲线重合
```

全部通过 → 系统状态正常
任一异常 → 按上述流程定位问题
