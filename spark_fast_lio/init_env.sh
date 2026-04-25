#!/bin/bash

# SPARK-FAST-LIO 环境初始化脚本

# 项目路径
PROJECT_DIR="/home/jerett/OpenProject/LidarSlam/spark-fast-lio"

# ============================================
# Conda 环境处理
# ============================================

# 检查是否在 conda 环境中
if [[ -n "$CONDA_PREFIX" ]]; then
    echo "检测到当前 Conda 环境: $CONDA_DEFAULT_ENV"
    echo "正在退出 Conda 环境..."
    conda deactivate
fi

# 可选: 激活指定的 conda 环境 (取消注释以启用)
# TARGET_CONDA_ENV="spark_fast_lio"  # 修改为你想要的环境名
# if conda env list | grep -q "^$TARGET_CONDA_ENV "; then
#     echo "激活 Conda 环境: $TARGET_CONDA_ENV"
#     conda activate $TARGET_CONDA_ENV
# else
#     echo "警告: Conda 环境 '$TARGET_CONDA_ENV' 不存在，使用系统 Python"
# fi

# 或者: 完全使用系统 Python (推荐用于 ROS2)
echo "使用系统 Python 环境"
export PATH="/usr/bin:$PATH"

# ============================================
# ROS2 环境设置
# ============================================

# ROS2 环境设置 (ROS2 Humble)
source /opt/ros/humble/setup.bash

# 如果存在 workspace 的 install 目录，也 source
if [ -f "$PROJECT_DIR/install/setup.bash" ]; then
    source $PROJECT_DIR/install/setup.bash
fi

echo "=========================================="
echo "SPARK-FAST-LIO Environment Initialized"
echo "Project Dir: $PROJECT_DIR"
echo "Python: $(which python3)"
echo "Conda Env: ${CONDA_DEFAULT_ENV:-系统环境}"
echo "=========================================="

# 编译函数
build() {
    echo "Building SPARK-FAST-LIO..."
    cd $PROJECT_DIR

    # 临时保存环境变量
    local _OLD_PYTHONPATH="$PYTHONPATH"
    local _OLD_PATH="$PATH"

    # 只保留 ROS2 的 Python 路径，移除 anaconda 路径
    export PYTHONPATH="/opt/ros/humble/lib/python3.10/site-packages"
    export PATH="/usr/bin:$PATH"

    # 强制 CMake 使用系统 Python
    colcon build --packages-up-to spark_fast_lio \
        --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3

    # 恢复环境变量
    export PYTHONPATH="$_OLD_PYTHONPATH"
    export PATH="$_OLD_PATH"

    source $PROJECT_DIR/install/setup.bash
    echo "Build complete!"
}

# 如果带参数 build，则执行编译
if [ "$1" == "build" ]; then
    build
fi
