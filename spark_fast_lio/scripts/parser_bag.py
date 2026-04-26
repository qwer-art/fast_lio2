import os
import csv
import json
import numpy as np
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Pose, PoseStamped, Vector3Stamped
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Float64MultiArray

#### 最多20hz，间隔50ms,重复的取一个就行
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


def parse_all_topics(bag_path, save_path):
    """解析bag文件中的所有指定话题，输出JSON文件

    输出:
    1. odometry/ 目录: 每个 frame_time 一个 JSON 文件
    2. path/ 目录: 每个 frame_time 一个 JSON 文件
    3. tf/ 目录: 每个 frame_time 一个 JSON 文件
    4. tf_static/ 目录: 每个 frame_time 一个 JSON 文件
    5. debug_data/ 目录: 每个 frame_time 一个 JSON 文件（包含所有debug topic）

    Args:
        bag_path: bag文件目录路径
        save_path: 输出目录路径
    """
    # 创建输出目录
    odometry_dir = os.path.join(save_path, "odometry")
    path_dir = os.path.join(save_path, "path")
    tf_dir = os.path.join(save_path, "tf")
    tf_static_dir = os.path.join(save_path, "tf_static")
    debug_dir = os.path.join(save_path, "debug_data")

    for d in [odometry_dir, path_dir, tf_dir, tf_static_dir, debug_dir]:
        os.makedirs(d, exist_ok=True)

    # 打开bag
    storage_opts = StorageOptions(uri=bag_path, storage_id="sqlite3")
    conv_opts = ConverterOptions("", "")
    reader = SequentialReader()
    reader.open(storage_opts, conv_opts)

    # 用于统计
    count_odometry = 0
    count_path = 0
    count_tf = 0
    count_tf_static = 0
    count_debug = 0

    msg_count = 0

    while reader.has_next():
        topic, data, t = reader.read_next()

        if topic == "/odometry":
            msg = deserialize_message(data, Odometry)
            timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            frame_time = timestamp_to_frametime(timestamp)

            odometry_item = {
                "frame_time": frame_time,
                "timestamp": timestamp,
                "pose": {
                    "position": {
                        "x": msg.pose.pose.position.x,
                        "y": msg.pose.pose.position.y,
                        "z": msg.pose.pose.position.z
                    },
                    "orientation": {
                        "x": msg.pose.pose.orientation.x,
                        "y": msg.pose.pose.orientation.y,
                        "z": msg.pose.pose.orientation.z,
                        "w": msg.pose.pose.orientation.w
                    },
                    "covariance": list(msg.pose.covariance)
                },
                "twist": {
                    "linear": {
                        "x": msg.twist.twist.linear.x,
                        "y": msg.twist.twist.linear.y,
                        "z": msg.twist.twist.linear.z
                    },
                    "angular": {
                        "x": msg.twist.twist.angular.x,
                        "y": msg.twist.twist.angular.y,
                        "z": msg.twist.twist.angular.z
                    },
                    "covariance": list(msg.twist.covariance)
                }
            }

            # 保存为单独的JSON文件
            json_file = os.path.join(odometry_dir, f"{frame_time}.json")
            with open(json_file, "w") as f:
                json.dump(odometry_item, f, indent=2)
            count_odometry += 1

        elif topic == "/path":
            msg = deserialize_message(data, Path)
            timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            frame_time = timestamp_to_frametime(timestamp)

            poses = []
            for pose in msg.poses:
                poses.append({
                    "position": {
                        "x": pose.pose.position.x,
                        "y": pose.pose.position.y,
                        "z": pose.pose.position.z
                    },
                    "orientation": {
                        "x": pose.pose.orientation.x,
                        "y": pose.pose.orientation.y,
                        "z": pose.pose.orientation.z,
                        "w": pose.pose.orientation.w
                    }
                })

            path_item = {
                "frame_time": frame_time,
                "timestamp": timestamp,
                "poses": poses
            }

            # 保存为单独的JSON文件
            json_file = os.path.join(path_dir, f"{frame_time}.json")
            with open(json_file, "w") as f:
                json.dump(path_item, f, indent=2)
            count_path += 1

        elif topic == "/tf":
            msg = deserialize_message(data, TFMessage)
            timestamp = t * 1e-9  # 使用bag时间戳
            frame_time = timestamp_to_frametime(timestamp)

            transforms = []
            for transform in msg.transforms:
                transforms.append({
                    "child_frame_id": transform.child_frame_id,
                    "header": {
                        "frame_id": transform.header.frame_id,
                        "timestamp": transform.header.stamp.sec + transform.header.stamp.nanosec * 1e-9
                    },
                    "transform": {
                        "translation": {
                            "x": transform.transform.translation.x,
                            "y": transform.transform.translation.y,
                            "z": transform.transform.translation.z
                        },
                        "rotation": {
                            "x": transform.transform.rotation.x,
                            "y": transform.transform.rotation.y,
                            "z": transform.transform.rotation.z,
                            "w": transform.transform.rotation.w
                        }
                    }
                })

            tf_item = {
                "frame_time": frame_time,
                "timestamp": timestamp,
                "transforms": transforms
            }

            # 保存为单独的JSON文件
            json_file = os.path.join(tf_dir, f"{frame_time}.json")
            with open(json_file, "w") as f:
                json.dump(tf_item, f, indent=2)
            count_tf += 1

        elif topic == "/tf_static":
            msg = deserialize_message(data, TFMessage)
            timestamp = t * 1e-9
            frame_time = timestamp_to_frametime(timestamp)

            transforms = []
            for transform in msg.transforms:
                transforms.append({
                    "child_frame_id": transform.child_frame_id,
                    "header": {
                        "frame_id": transform.header.frame_id,
                        "timestamp": transform.header.stamp.sec + transform.header.stamp.nanosec * 1e-9
                    },
                    "transform": {
                        "translation": {
                            "x": transform.transform.translation.x,
                            "y": transform.transform.translation.y,
                            "z": transform.transform.translation.z
                        },
                        "rotation": {
                            "x": transform.transform.rotation.x,
                            "y": transform.transform.rotation.y,
                            "z": transform.transform.rotation.z,
                            "w": transform.transform.rotation.w
                        }
                    }
                })

            tf_static_item = {
                "frame_time": frame_time,
                "timestamp": timestamp,
                "transforms": transforms
            }

            # 保存为单独的JSON文件
            json_file = os.path.join(tf_static_dir, f"{frame_time}.json")
            with open(json_file, "w") as f:
                json.dump(tf_static_item, f, indent=2)
            count_tf_static += 1

        elif topic in ["/debug/delta_pose", "/debug/imu_bias_acc", "/debug/imu_bias_gyro",
                       "/debug/imu_preint_pose", "/debug/match_quality", "/debug/velocity"]:
            # 对于debug topic，需要合并同一个frame_time的数据
            if topic == "/debug/delta_pose":
                msg = deserialize_message(data, Pose)
                timestamp = t * 1e-9
                frame_time = timestamp_to_frametime(timestamp)
                debug_item = {
                    "frame_time": frame_time,
                    "timestamp": timestamp,
                    "delta_pose": {
                        "position": {
                            "x": msg.position.x,
                            "y": msg.position.y,
                            "z": msg.position.z
                        },
                        "orientation": {
                            "x": msg.orientation.x,
                            "y": msg.orientation.y,
                            "z": msg.orientation.z,
                            "w": msg.orientation.w
                        }
                    }
                }

            elif topic == "/debug/imu_bias_acc":
                msg = deserialize_message(data, Vector3Stamped)
                timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                frame_time = timestamp_to_frametime(timestamp)
                debug_item = {
                    "frame_time": frame_time,
                    "timestamp": timestamp,
                    "imu_bias_acc": {
                        "x": msg.vector.x,
                        "y": msg.vector.y,
                        "z": msg.vector.z
                    }
                }

            elif topic == "/debug/imu_bias_gyro":
                msg = deserialize_message(data, Vector3Stamped)
                timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                frame_time = timestamp_to_frametime(timestamp)
                debug_item = {
                    "frame_time": frame_time,
                    "timestamp": timestamp,
                    "imu_bias_gyro": {
                        "x": msg.vector.x,
                        "y": msg.vector.y,
                        "z": msg.vector.z
                    }
                }

            elif topic == "/debug/imu_preint_pose":
                msg = deserialize_message(data, PoseStamped)
                timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                frame_time = timestamp_to_frametime(timestamp)
                debug_item = {
                    "frame_time": frame_time,
                    "timestamp": timestamp,
                    "imu_preint_pose": {
                        "position": {
                            "x": msg.pose.position.x,
                            "y": msg.pose.position.y,
                            "z": msg.pose.position.z
                        },
                        "orientation": {
                            "x": msg.pose.orientation.x,
                            "y": msg.pose.orientation.y,
                            "z": msg.pose.orientation.z,
                            "w": msg.pose.orientation.w
                        }
                    }
                }

            elif topic == "/debug/match_quality":
                msg = deserialize_message(data, Float64MultiArray)
                timestamp = t * 1e-9
                frame_time = timestamp_to_frametime(timestamp)
                debug_item = {
                    "frame_time": frame_time,
                    "timestamp": timestamp,
                    "match_quality": list(msg.data)
                }

            elif topic == "/debug/velocity":
                msg = deserialize_message(data, Vector3Stamped)
                timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                frame_time = timestamp_to_frametime(timestamp)
                debug_item = {
                    "frame_time": frame_time,
                    "timestamp": timestamp,
                    "velocity": {
                        "x": msg.vector.x,
                        "y": msg.vector.y,
                        "z": msg.vector.z
                    }
                }

            # 对于debug数据，需要合并同一个frame_time的多个字段
            json_file = os.path.join(debug_dir, f"{frame_time}.json")

            # 如果文件已存在，读取并合并
            if os.path.exists(json_file):
                with open(json_file, "r") as f:
                    existing_data = json.load(f)
                # 合并数据
                existing_data.update(debug_item)
                debug_item = existing_data

            # 保存更新后的数据
            with open(json_file, "w") as f:
                json.dump(debug_item, f, indent=2)
            count_debug += 1

        msg_count += 1
        if msg_count % 10000 == 0:
            print(f"  已处理 {msg_count} 条消息...")

    print(f"\n完成! 共处理 {msg_count} 条消息")
    print(f"  odometry/: {count_odometry} 个文件")
    print(f"  path/: {count_path} 个文件")
    print(f"  tf/: {count_tf} 个文件")
    print(f"  tf_static/: {count_tf_static} 个文件")
    print(f"  debug_data/: {count_debug} 个数据点（合并到多个文件中）")


if __name__ == "__main__":
    bag_path = "/home/jerett/OpenProject/LidarSlam/spark-fast-lio/spark_fast_lio/scripts/data/lio_20260426_174848"
    save_path = "/home/jerett/OpenProject/LidarSlam/spark-fast-lio/spark_fast_lio/scripts/data/lio_20260426_174848/asset_data"

    parse_all_topics(bag_path, save_path)
