# SPARK-FAST-LIO 系统架构图

## 1. 系统整体架构

```mermaid
graph TB
    subgraph 传感器层["传感器层 (Sensor Layer)"]
        LIDAR[LiDAR传感器<br/>Velodyne/Ouster/Livox]
        IMU[IMU传感器<br/>6轴惯性测量单元]
    end

    subgraph 数据采集层["数据采集层 (Data Collection)"]
        LIDAR_CB[LiDAR回调<br/>standardLiDARCallback]
        IMU_CB[IMU回调<br/>imuCallback]
    end

    subgraph 数据缓冲层["数据缓冲层 (Buffer Layer)"]
        LIDAR_BUF[点云缓冲队列<br/>lidar_buffer_]
        IMU_BUF[IMU缓冲队列<br/>imu_buffer_]
        TIME_BUF[时间戳缓冲<br/>time_buffer_]
    end

    subgraph 数据同步层["数据同步层 (Sync Layer)"]
        SYNC[syncPackages<br/>MeasureGroup构建]
    end

    subgraph 预处理层["预处理层 (Preprocessing)"]
        PREPROCESS[点云预处理<br/>Preprocess::process]
        IMU_PROCESS[IMU处理<br/>ImuProcess::Process]
        DESKEW[点云去畸变<br/>Deskew Pointcloud]
    end

    subgraph 状态估计层["状态估计层 (State Estimation)"]
        IKFOM[迭代EKF滤波器<br/>IKFoM Toolkit]
        STATE_UPDATE[状态更新<br/>迭代优化]
        COV_UPDATE[协方差更新<br/>P矩阵传播]
    end

    subgraph 地图管理层["地图管理层 (Map Management)"]
        IKD_TREE[增量式KD树<br/>ikd-Tree]
        LOCAL_MAP[局部地图管理<br/>Local Map]
        MAP_UPDATE[地图增量更新<br/>mapIncremental]
    end

    subgraph 结果发布层["结果发布层 (Output Layer)"]
        ODOM[里程计发布<br/>Odometry]
        PATH[路径发布<br/>Path]
        CLOUD[点云发布<br/>Point Cloud]
        TF[TF变换发布<br/>Transform]
    end

    %% 数据流连接
    LIDAR --> LIDAR_CB
    IMU --> IMU_CB
    LIDAR_CB --> LIDAR_BUF
    IMU_CB --> IMU_BUF
    LIDAR_BUF --> SYNC
    IMU_BUF --> SYNC
    TIME_BUF --> SYNC

    SYNC --> PREPROCESS
    SYNC --> IMU_PROCESS
    PREPROCESS --> DESKEW
    IMU_PROCESS --> DESKEW

    DESKEW --> IKFOM
    IKFOM --> STATE_UPDATE
    STATE_UPDATE --> COV_UPDATE

    STATE_UPDATE --> IKD_TREE
    IKD_TREE --> LOCAL_MAP
    LOCAL_MAP --> MAP_UPDATE

    MAP_UPDATE --> ODOM
    MAP_UPDATE --> PATH
    MAP_UPDATE --> CLOUD
    MAP_UPDATE --> TF

    %% 样式
    classDef sensor fill:#e1f5ff,stroke:#01579b,stroke-width:2px
    classDef buffer fill:#fff3e0,stroke:#e65100,stroke-width:2px
    classDef process fill:#f3e5f5,stroke:#4a148c,stroke-width:2px
    classDef filter fill:#e8f5e9,stroke:#1b5e20,stroke-width:2px
    classDef output fill:#fff9c4,stroke:#f57f17,stroke-width:2px

    class LIDAR,IMU sensor
    class LIDAR_BUF,IMU_BUF,TIME_BUF buffer
    class PREPROCESS,IMU_PROCESS,DESKEW,SYNC process
    class IKFOM,STATE_UPDATE,COV_UPDATE,IKD_TREE,LOCAL_MAP,MAP_UPDATE filter
    class ODOM,PATH,CLOUD,TF output
```

## 2. 核心类关系图

```mermaid
classDiagram
    class SPARKFastLIO2 {
        -ImuProcess imu_processor_
        -Preprocess preprocessor_
        -esekfom kf_
        -PointCloudXYZI feats_undistort_
        -std::deque lidar_buffer_
        -std::deque imu_buffer_
        +syncPackages()
        +processLidarAndImu()
        +mapIncremental()
        +publishOdometry()
    }

    class ImuProcess {
        -state_ikfom state_
        -input_ikfom input_
        -double last_lidar_end_time_
        +Process()
        +IMU_init()
        +ForwardPropagation()
        +BackwardPropagation()
    }

    class Preprocess {
        -int lidar_type_
        -double blind_
        +process()
        +livoxHandler()
        +velodyneHandler()
        +ousterHandler()
    }

    class esekfom {
        -state_ikfom x_
        -Eigen::Matrix P_
        +update_iterated_dyn_share_modified()
        +predict()
    }

    class state_ikfom {
        +vect3 pos
        +SO3 rot
        +SO3 offset_R_L_I
        +vect3 offset_T_L_I
        +vect3 vel
        +vect3 bg
        +vect3 ba
        +S2 grav
    }

    SPARKFastLIO2 --> ImuProcess
    SPARKFastLIO2 --> Preprocess
    SPARKFastLIO2 --> esekfom
    ImuProcess --> state_ikfom
    esekfom --> state_ikfom

    note for SPARKFastLIO2 "主控制类\n继承自 rclcpp::Node"
    note for ImuProcess "IMU数据处理\n状态预测与积分"
    note for Preprocess "点云预处理\n多传感器支持"
    note for esekfom "迭代EKF滤波器\n状态估计核心"
```

## 3. 数据流详细图

```mermaid
sequenceDiagram
    participant LiDAR as LiDAR传感器
    participant IMU as IMU传感器
    participant Buffer as 数据缓冲
    participant Sync as 数据同步
    participant Preproc as 预处理
    participant EKF as 迭代EKF
    participant Map as 地图管理
    participant Output as 结果发布

    LiDAR->>Buffer: 点云数据 (10Hz)
    IMU->>Buffer: IMU数据 (200Hz)

    loop 主循环
        Buffer->>Sync: 提取同步数据包
        Sync->>Preproc: MeasureGroup

        Preproc->>Preproc: 点云去畸变
        Preproc->>Preproc: 降采样

        Preproc->>EKF: 预测状态
        EKF->>Map: 最近邻搜索
        Map-->>EKF: 返回最近点
        EKF->>EKF: 迭代更新

        EKF->>Map: 更新地图
        EKF->>Output: 发布结果
    end
```

## 4. 状态估计流程图

```mermaid
flowchart TD
    START([开始]) --> INIT{是否已初始化?}

    INIT -->|否| GRAVITY[重力对齐<br/>估计重力方向]
    GRAVITY --> BIAS[零偏估计<br/>陀螺仪/加速度计]
    BIAS --> INIT_DONE[初始化完成]

    INIT -->|是| SYNC[数据同步<br/>syncPackages]
    INIT_DONE --> SYNC

    SYNC --> PREDICT[状态预测<br/>IMU前向传播]
    PREDICT --> DESKEW[点云去畸变<br/>运动补偿]

    DESKEW --> DOWNSAMPLE[点云降采样<br/>体素滤波]
    DOWNSAMPLE --> NN_SEARCH[最近邻搜索<br/>ikd-Tree]

    NN_SEARCH --> RESIDUAL[计算残差<br/>点到面距离]
    RESIDUAL --> JACOBIAN[构建雅可比矩阵<br/>∂h/∂x]

    JACOBIAN --> ITERATE{迭代收敛?}
    ITERATE -->|否| UPDATE[状态更新<br/>x = x ⊞ δx]
    UPDATE --> RESIDUAL

    ITERATE -->|是| COV[协方差更新<br/>P = FPFᵀ + Q]
    COV --> MAP_UPDATE[地图更新<br/>ikd-Tree增量添加]

    MAP_UPDATE --> PUBLISH[发布结果<br/>Odometry/Path/TF]
    PUBLISH --> END([结束])

    style START fill:#e1f5ff
    style END fill:#e1f5ff
    style INIT fill:#fff3e0
    style ITERATE fill:#fff3e0
    style GRAVITY fill:#f3e5f5
    style BIAS fill:#f3e5f5
    style PREDICT fill:#e8f5e9
    style RESIDUAL fill:#e8f5e9
    style JACOBIAN fill:#e8f5e9
```

## 5. 模块依赖关系图

```mermaid
graph LR
    subgraph ROS2["ROS2 框架"]
        NODE[rclcpp::Node]
        SUB[Subscription]
        PUB[Publisher]
        TF2[TF2 Broadcaster]
    end

    subgraph Core["核心算法"]
        IKFOM[IKFoM Toolkit]
        IKD[ikd-Tree]
        IMU_PROC[IMU Processing]
        PREPROC[Preprocessing]
    end

    subgraph Math["数学库"]
        SO3[SO3 Math]
        EXP[Exponential Map]
        EIGEN[Eigen]
    end

    subgraph Utils["工具库"]
        PCL[PCL]
        OpenMP[OpenMP]
        SmartPtr[Smart Pointers]
    end

    SPARK[SPARKFastLIO2] --> NODE
    SPARK --> IKFOM
    SPARK --> IKD
    SPARK --> IMU_PROC
    SPARK --> PREPROC

    IKFOM --> SO3
    IKFOM --> EXP
    IKFOM --> EIGEN

    IMU_PROC --> SO3
    IMU_PROC --> EIGEN

    IKD --> EIGEN
    IKD --> OpenMP

    PREPROC --> PCL
    PREPROC --> EIGEN

    SPARK --> SUB
    SPARK --> PUB
    SPARK --> TF2

    style SPARK fill:#ff6b6b,stroke:#c92a2a,stroke-width:3px
    style IKFOM fill:#4ecdc4,stroke:#087f5b,stroke-width:2px
    style IKD fill:#4ecdc4,stroke:#087f5b,stroke-width:2px
```

## 6. 文件组织结构图

```
spark_fast_lio/
├── src/                              # 源文件
│   ├── spark_fast_lio.cpp            # 主节点实现
│   │   ├── SPARKFastLIO2类           # 主控制类
│   │   ├── standardLiDARCallback()   # LiDAR回调
│   │   ├── imuCallback()             # IMU回调
│   │   ├── syncPackages()            # 数据同步
│   │   ├── processLidarAndImu()      # 主处理循环
│   │   ├── mapIncremental()          # 地图更新
│   │   └── publishOdometry()         # 结果发布
│   │
│   └── preprocess.cpp                # 点云预处理实现
│       ├── livoxHandler()            # Livox处理
│       ├── velodyneHandler()         # Velodyne处理
│       └── ousterHandler()           # Ouster处理
│
├── include/spark_fast_lio/           # 头文件
│   ├── spark_fast_lio.h              # 主类定义
│   ├── imu_processing.hpp            # IMU处理类
│   ├── preprocess.h                  # 预处理接口
│   │
│   └── common/                       # 公共库
│       ├── common_lib.h              # 数据结构定义
│       │   ├── PointType             # 点类型定义
│       │   ├── MeasureGroup          # 测量数据组
│       │   └── StatesGroup           # 状态组
│       ├── use-ikfom.hpp             # 状态流形定义
│       │   ├── state_ikfom           # 23维状态向量
│       │   ├── input_ikfom           # 输入向量
│       │   └── get_f()               # 状态转移函数
│       ├── so3_math.h                # SO3数学运算
│       └── Exp_mat.h                 # 指数映射
│
├── third_party/                      # 第三方库
│   ├── IKFoM_toolkit/                # 迭代卡尔曼滤波工具包
│   │   ├── esekfom/                  # ESEKF实现
│   │   │   ├── esekfom.hpp           # 滤波器核心
│   │   │   └── esekfom_impl.hpp      # 实现细节
│   │   └── mtk/                      # 流形工具包
│   │       ├── src/SubManifold.hpp   # 子流形
│   │       ├── src/BuildManifold.hpp # 流形构建
│   │       └── src/math.hpp          # 数学运算
│   │
│   └── ikd-Tree/                     # 增量式KD树
│       ├── ikd-Tree.cpp              # KD树实现
│       ├── ikd-Tree.h                # 接口定义
│       └── README.md                 # 文档
│
├── config/                           # 配置文件
│   ├── velodyne.yaml                 # Velodyne配置
│   ├── ouster.yaml                   # Ouster配置
│   └── livox.yaml                    # Livox配置
│
├── launch/                           # 启动文件
│   ├── mapping_velodyne.launch.py    # Velodyne启动
│   ├── mapping_ouster.launch.py      # Ouster启动
│   └── mapping_livox.launch.py       # Livox启动
│
└── rviz/                             # 可视化配置
    └── spark_fast_lio.rviz           # RViz配置文件
```

## 7. 关键算法流程

### 7.1 迭代EKF更新流程

```mermaid
flowchart LR
    subgraph 预测["预测步骤"]
        P1[IMU积分] --> P2[状态传播]
        P2 --> P3[协方差传播]
    end

    subgraph 更新["更新步骤"]
        U1[最近邻搜索] --> U2[计算残差]
        U2 --> U3[构建雅可比]
        U3 --> U4[卡尔曼增益]
        U4 --> U5[状态更新]
        U5 --> U6{收敛?}
        U6 -->|否| U2
        U6 -->|是| U7[协方差更新]
    end

    预测 --> 更新

    style P1 fill:#e3f2fd
    style U1 fill:#fff3e0
    style U7 fill:#e8f5e9
```

### 7.2 点云去畸变流程

```mermaid
flowchart TD
    RAW[原始点云] --> TIME[提取时间戳]
    TIME --> IMU_INT[IMU状态积分]
    IMU_INT --> ROT[计算旋转补偿]
    ROT --> TRANS[计算平移补偿]
    TRANS --> COMP[应用变换矩阵]
    COMP --> DESKEWED[去畸变点云]

    style RAW fill:#e1f5ff
    style DESKEWED fill:#e8f5e9
```

## 8. ROS2话题通信图

```mermaid
graph TB
    subgraph 输入话题["输入话题 (Input Topics)"]
        LIDAR_IN["/lidar<br/>PointCloud2"]
        LIVOX_IN["/lidar<br/>CustomMsg<br/>(Livox)"]
        IMU_IN["/imu<br/>Imu"]
    end

    subgraph 节点["SPARK-Fast-LIO 节点"]
        CORE[核心处理]
    end

    subgraph 输出话题["输出话题 (Output Topics)"]
        ODOM["/odometry<br/>Odometry"]
        PATH["/path<br/>Path"]
        CLOUD1["/cloud_registered<br/>PointCloud2"]
        CLOUD2["/cloud_registered_lidar<br/>PointCloud2"]
        CLOUD3["/cloud_registered_body<br/>PointCloud2"]
        CLOUD4["/cloud_registered_base<br/>PointCloud2"]
    end

    subgraph TF树["TF树"]
        MAP["map_frame<br/>(odom)"]
        CHILD["child_frame<br/>(imu/lidar/base)"]
    end

    LIDAR_IN --> CORE
    LIVOX_IN --> CORE
    IMU_IN --> CORE

    CORE --> ODOM
    CORE --> PATH
    CORE --> CLOUD1
    CORE --> CLOUD2
    CORE --> CLOUD3
    CORE --> CLOUD4

    CORE --> MAP
    MAP --> CHILD

    style CORE fill:#ff6b6b,stroke:#c92a2a,stroke-width:3px
```

## 9. 性能优化架构

```mermaid
graph TB
    subgraph 多线程优化["多线程优化"]
        OMP[OpenMP并行化]
        THREAD[多线程KD树重建]
    end

    subgraph 内存优化["内存优化"]
        PREALLOC[预分配内存]
        ALIGN[Eigen内存对齐]
        SMART[智能指针管理]
    end

    subgraph 数值优化["数值优化"]
        SO3_MANIFOLD[SO3流形表示]
        COV_POS[协方差正定性维护]
        OUTLIER[异常值剔除]
    end

    subgraph 算法优化["算法优化"]
        IKD[增量式KD树]
        ITER[迭代EKF]
        ADAPT[自适应降采样]
    end

    PERF[性能优化] --> 多线程优化
    PERF --> 内存优化
    PERF --> 数值优化
    PERF --> 算法优化

    style PERF fill:#ff6b6b,stroke:#c92a2a,stroke-width:3px
```

## 10. 系统配置架构

```mermaid
graph LR
    subgraph 传感器配置["传感器配置"]
        TYPE[lidar_type<br/>1:Livox 2:Velodyne 3:Ouster]
        SCAN[scan_line<br/>激光线数]
        TIME[timestamp_unit<br/>时间戳单位]
        BLIND[blind<br/>最小距离]
    end

    subgraph 滤波器配置["滤波器配置"]
        ACC[acc_cov<br/>加速度协方差]
        GYR[gyr_cov<br/>陀螺仪协方差]
        BIAS[b_acc_cov<br/>b_gyr_cov<br/>零偏协方差]
        EXTR[extrinsic_T<br/>extrinsic_R<br/>外参]
    end

    subgraph 地图配置["地图配置"]
        FOV[fov_degree<br/>视场角]
        RANGE[det_range<br/>探测距离]
        FILTER[filter_size_map<br/>体素大小]
    end

    subgraph 发布配置["发布配置"]
        PATH_EN[path_en]
        SCAN_EN[scan_publish_en]
        DENSE[dense_publish_en]
    end

    CONFIG[配置文件] --> 传感器配置
    CONFIG --> 滤波器配置
    CONFIG --> 地图配置
    CONFIG --> 发布配置

    style CONFIG fill:#4ecdc4,stroke:#087f5b,stroke-width:2px
```

---

## 关键技术特点总结

### 1. 紧耦合融合
- **IMU-LiDAR紧耦合**：在滤波器层面融合，而非简单的位姿融合
- **高频预测**：IMU提供高频(200Hz)状态预测
- **低频更新**：LiDAR提供低频(10Hz)观测更新

### 2. 迭代误差状态卡尔曼滤波
- **迭代更新**：每次观测进行多次迭代，提高非线性系统精度
- **误差状态**：在流形上进行误差状态传播，避免参数化奇异
- **协方差维护**：严格维护协方差矩阵的正定性

### 3. 增量式地图管理
- **ikd-Tree**：支持动态点插入和删除的增量式KD树
- **局部地图**：维护局部滑动窗口地图
- **高效搜索**：O(log n)复杂度的最近邻搜索

### 4. 多传感器支持
- **Livox**：支持Livox自定义消息格式
- **Velodyne**：支持标准Velodyne点云
- **Ouster**：支持Ouster激光雷达
- **自动检测**：根据配置自动选择处理方式

### 5. 工程优化
- **ROS2原生**：充分利用ROS2的优势
- **多线程**：OpenMP并行化加速
- **内存管理**：预分配和智能指针
- **数值稳定**：SO(3)流形和协方差正定性维护

---

**文档版本**: v1.0
**生成日期**: 2026-04-18
**适用版本**: SPARK-FAST-LIO (ROS2 Jazzy)
