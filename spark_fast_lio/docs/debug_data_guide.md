# LIO Debug Data 分析指南

## 数据文件

单一CSV文件 `logs/<timestamp>/lio_debug.csv`，所有数据按 `frame_time` 对齐。

## 字段说明

| 字段 | 含义 | 单位 |
|------|------|------|
| `timestamp` | 原始时间戳 | s |
| `frame_time` | 对齐时间戳(50ms网格) | s |
| `odom_x/y/z` | ESKF优化后位置 | m |
| `odom_roll/pitch/yaw` | ESKF优化后姿态 | rad |
| `preint_x/y/z` | IMU预积分预测位置 | m |
| `preint_roll/pitch/yaw` | IMU预积分预测姿态 | rad |
| `bg_x/y/z` | 陀螺仪Bias | rad/s |
| `ba_x/y/z` | 加速度计Bias | m/s² |
| `vx/vy/vz` | 速度 | m/s |
| `feat_num` | 有效特征点数 | - |
| `lidar_res` | Lidar匹配残差 | m |
| `imu_res` | IMU预积分残差 | m+rad |
| `solve_time` | 求解耗时 | s |
| `delta_x/y/z` | 帧间位置增量 | m |
| `delta_roll/pitch/yaw` | 帧间姿态增量 | rad |

---

## 诊断流程

### Step 1: 检查匹配质量

**关键字段**: `feat_num`, `lidar_res`

| 检查项 | 正常值 | 异常 → 可能原因 |
|--------|--------|----------------|
| feat_num | >500 | 点云质量差、环境空旷 |
| lidar_res | <0.1 | 匹配退化、特征单一 |
| imu_res | <0.5 | IMU漂移、参数标定误差 |

### Step 2: 检查IMU Bias收敛

**关键字段**: `bg_x/y/z`, `ba_x/y/z`

| 检查项 | 正常表现 | 异常 → 可能原因 |
|--------|----------|----------------|
| Bias曲线 | 快速收敛后稳定 | 发散: IMU异常 |
| 收敛时间 | <5s | 波动剧烈: 运动过激 |

### Step 3: 检查运动一致性

**关键字段**: `vx/vy/vz`, `odom_*` vs `preint_*`

| 检查项 | 正常表现 | 异常 → 可能原因 |
|--------|----------|----------------|
| 速度曲线 | 平滑无突变 | 突变: 数据不同步 |
| odom-preint差异 | 曲线重合 | 分离: IMU漂移累积 |

---

## PlotJuggler使用

1. **加载**: File → Load Data → 选择 `lio_debug.csv`
2. **时间轴**: 使用 `frame_time` 作为横轴

**推荐布局**:
- 上: `feat_num`, `lidar_res`, `imu_res` (匹配质量)
- 中: `bg_*`, `ba_*` (IMU状态)
- 下: `odom_x` vs `preint_x` (轨迹对比)

---

## 快速Checklist

```
□ feat_num > 500
□ lidar_res < 0.1
□ imu_res < 0.5
□ Bias收敛稳定
□ 速度平滑
□ odom与preint重合
```

**全部OK** → 系统正常  
**任一异常** → 按上述步骤定位