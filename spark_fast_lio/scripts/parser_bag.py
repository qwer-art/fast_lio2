import os
import csv
import numpy as np
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry


def timestamp_to_frametime(timestamp):
    """将时间戳转换为frame_time格式

    frame_time格式:
    - 前12位: 秒部分，不足补0
    - 后12位: 微秒部分，不足补0
    - 微秒按20Hz帧率(50ms=50000us)四舍五入

    Args:
        timestamp: float, 如 1666028921.372154951

    Returns:
        str: 24位frame_time字符串
    """
    sec = int(timestamp)
    us = round((timestamp - sec) * 1e6)

    # 按50ms(50000us)四舍五入
    frame_us = round(us / 50000) * 50000

    # 处理进位: 如果微秒部分进位到1秒
    if frame_us >= 1000000:
        sec += frame_us // 1000000
        frame_us = frame_us % 1000000

    # 前12位秒，下划线分隔，后12位微秒
    return f"{sec:012d}_{frame_us:012d}"


def parse_odometry(bag_path, save_path):
    """解析bag文件中的/odometry话题，输出3类CSV

    输出:
    1. state.csv: timestamp, pos_x, pos_y, pos_z, ori_x, ori_y, ori_z, ori_w,
                  lin_x, lin_y, lin_z, ang_x, ang_y, ang_z
    2. pose_cov/<frame_time>.csv: 每个时间戳一个6x6协方差矩阵CSV
    3. twist_cov/<frame_time>.csv: 每个时间戳一个6x6协方差矩阵CSV

    协方差矩阵布局 (6x6, 行列顺序: x, y, z, rx, ry, rz):
           x    y    z   rx   ry   rz
        ┌────────────────────────────┐
  x     │  0   1   2   3   4   5    │
  y     │  6   7   8   9  10  11    │
  z     │ 12  13  14  15  16  17    │
  rx    │ 18  19  20  21  22  23    │
  ry    │ 24  25  26  27  28  29    │
  rz    │ 30  31  32  33  34  35    │
        └────────────────────────────┘

    Args:
        bag_path: bag文件目录路径
        save_path: 输出目录路径
    """
    # 创建输出目录
    pose_cov_dir = os.path.join(save_path, "pose_cov")
    twist_cov_dir = os.path.join(save_path, "twist_cov")
    os.makedirs(pose_cov_dir, exist_ok=True)
    os.makedirs(twist_cov_dir, exist_ok=True)

    # 打开bag
    storage_opts = StorageOptions(uri=bag_path, storage_id="sqlite3")
    conv_opts = ConverterOptions("", "")
    reader = SequentialReader()
    reader.open(storage_opts, conv_opts)

    # 准备state.csv
    state_path = os.path.join(save_path, "state.csv")
    state_header = [
        "timestamp", "pos_x", "pos_y", "pos_z",
        "ori_x", "ori_y", "ori_z", "ori_w",
        "lin_x", "lin_y", "lin_z",
        "ang_x", "ang_y", "ang_z",
    ]
    cov_row_labels = ["x", "y", "z", "rx", "ry", "rz"]
    cov_col_labels = ["x", "y", "z", "rx", "ry", "rz"]

    msg_count = 0

    with open(state_path, "w", newline="") as f_state:
        writer = csv.writer(f_state)
        writer.writerow(state_header)

        while reader.has_next():
            topic, data, t = reader.read_next()
            if topic != "/odometry":
                continue

            msg = deserialize_message(data, Odometry)

            # 时间戳
            timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            frame_time = timestamp_to_frametime(timestamp)

            # state行
            p = msg.pose.pose.position
            o = msg.pose.pose.orientation
            lin = msg.twist.twist.linear
            ang = msg.twist.twist.angular
            writer.writerow([
                f"{timestamp:.9f}",
                p.x, p.y, p.z,
                o.x, o.y, o.z, o.w,
                lin.x, lin.y, lin.z,
                ang.x, ang.y, ang.z,
            ])

            # pose_cov: 36元素 -> 6x6矩阵
            pose_cov = np.array(msg.pose.covariance).reshape(6, 6)
            pose_cov_file = os.path.join(pose_cov_dir, f"{frame_time}.csv")
            with open(pose_cov_file, "w", newline="") as f_cov:
                cov_writer = csv.writer(f_cov)
                cov_writer.writerow([""] + cov_col_labels)
                for i, label in enumerate(cov_row_labels):
                    cov_writer.writerow([label] + pose_cov[i].tolist())

            # twist_cov: 36元素 -> 6x6矩阵
            twist_cov = np.array(msg.twist.covariance).reshape(6, 6)
            twist_cov_file = os.path.join(twist_cov_dir, f"{frame_time}.csv")
            with open(twist_cov_file, "w", newline="") as f_cov:
                cov_writer = csv.writer(f_cov)
                cov_writer.writerow([""] + cov_col_labels)
                for i, label in enumerate(cov_row_labels):
                    cov_writer.writerow([label] + twist_cov[i].tolist())

            msg_count += 1
            if msg_count % 10000 == 0:
                print(f"  已处理 {msg_count} 条odometry消息...")

    print(f"完成! 共处理 {msg_count} 条odometry消息")
    print(f"  state.csv: {state_path}")
    print(f"  pose_cov/: {pose_cov_dir}/ ({msg_count} 个文件)")
    print(f"  twist_cov/: {twist_cov_dir}/ ({msg_count} 个文件)")


if __name__ == "__main__":
    bag_path = "/home/jerett/OpenProject/LidarSlam/spark-fast-lio/spark_fast_lio/scripts/data/lio_20260426_142833"
    save_path = "/home/jerett/OpenProject/LidarSlam/spark-fast-lio/spark_fast_lio/scripts/data/lio_20260426_142833/asset_data"

    parse_odometry(bag_path, save_path)
