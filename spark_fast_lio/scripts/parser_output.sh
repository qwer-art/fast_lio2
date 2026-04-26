#!/bin/bash
# Output数据解析脚本
# 调用parser_output.py将output数据整理成CSV文件

# ============================================
# 路径配置
# ============================================
asset_data=/home/jerett/OpenProject/LidarSlam/spark-fast-lio/spark_fast_lio/data/asset_data
ekf=/home/jerett/OpenProject/LidarSlam/spark-fast-lio/spark_fast_lio/data/ekf

# ============================================
# 执行解析
# ============================================
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
python3 "${SCRIPT_DIR}/parser_output.py" "$asset_data" "$ekf"
