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
from sensor_msgs.msg import Imu

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
    imu_dir = os.path.join(save_path, "imu")
    state_other_dir = os.path.join(save_path, "state_other")

    for d in [odometry_dir, path_dir, tf_dir, tf_static_dir, debug_dir, imu_dir, state_other_dir]:
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
    count_imu = 0
    count_state_other = 0

    msg_count = 0

    while reader.has_next():
        topic, data, t = reader.read_next()

        if topic == "/odometry":
            msg = deserialize_message(data, Odometry)
            # 使用bag时间戳来同步，而不是msg.header.stamp
            # 因为不同消息的header.stamp可能来自不同的时钟源
            timestamp = t * 1e-9
            frame_time = timestamp_to_frametime(timestamp)

            # 保存原始header时间戳用于参考
            header_timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

            odometry_item = {
                "frame_time": frame_time,
                "timestamp": timestamp,
                "header_timestamp": header_timestamp,
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

        elif topic == "/hathor/forward/imu":
            msg = deserialize_message(data, Imu)
            timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            frame_time = timestamp_to_frametime(timestamp)

            imu_item = {
                "frame_time": frame_time,
                "timestamp": timestamp,
                "linear_acceleration": {
                    "x": msg.linear_acceleration.x,
                    "y": msg.linear_acceleration.y,
                    "z": msg.linear_acceleration.z
                },
                "angular_velocity": {
                    "x": msg.angular_velocity.x,
                    "y": msg.angular_velocity.y,
                    "z": msg.angular_velocity.z
                },
                "orientation": {
                    "x": msg.orientation.x,
                    "y": msg.orientation.y,
                    "z": msg.orientation.z,
                    "w": msg.orientation.w
                }
            }

            # 保存为单独的JSON文件
            json_file = os.path.join(imu_dir, f"{frame_time}.json")
            with open(json_file, "w") as f:
                json.dump(imu_item, f, indent=2)
            count_imu += 1

        elif topic == "/state_other":
            msg = deserialize_message(data, Float64MultiArray)
            # 数据布局: [bg_x, bg_y, bg_z, ba_x, ba_y, ba_z,
            #           offset_R_x, offset_R_y, offset_R_z, offset_T_x, offset_T_y, offset_T_z,
            #           grav_x, grav_y, grav_z]
            if len(msg.data) >= 15:
                # 使用odometry的时间戳近似（state_other没有header）
                timestamp = t * 1e-9
                frame_time = timestamp_to_frametime(timestamp)

                state_other_item = {
                    "frame_time": frame_time,
                    "timestamp": timestamp,
                    "bg": {
                        "x": msg.data[0],
                        "y": msg.data[1],
                        "z": msg.data[2]
                    },
                    "ba": {
                        "x": msg.data[3],
                        "y": msg.data[4],
                        "z": msg.data[5]
                    },
                    "offset_R_L_I": {
                        "x": msg.data[6],
                        "y": msg.data[7],
                        "z": msg.data[8]
                    },
                    "offset_T_L_I": {
                        "x": msg.data[9],
                        "y": msg.data[10],
                        "z": msg.data[11]
                    },
                    "grav": {
                        "x": msg.data[12],
                        "y": msg.data[13],
                        "z": msg.data[14]
                    }
                }

                json_file = os.path.join(state_other_dir, f"{frame_time}.json")
                with open(json_file, "w") as f:
                    json.dump(state_other_item, f, indent=2)
                count_state_other += 1

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
    print(f"  imu/: {count_imu} 个文件")
    print(f"  state_other/: {count_state_other} 个文件")
    print(f"  path/: {count_path} 个文件")
    print(f"  tf/: {count_tf} 个文件")
    print(f"  tf_static/: {count_tf_static} 个文件")
    print(f"  debug_data/: {count_debug} 个数据点（合并到多个文件中）")


def export_to_csv(bag_path, save_path):
    """将关键数据导出为CSV格式，便于PlotJuggler可视化

    输出:
    1. state.csv: 位置、姿态、速度、角速度
    2. imu_debug.csv: IMU零偏、预积分位姿、速度
    3. covariance.csv: 协方差对角线元素（位置、姿态、速度的不确定性）
    4. match_quality.csv: 匹配质量指标

    Args:
        bag_path: bag文件目录路径
        save_path: 输出目录路径
    """
    # 创建输出目录
    os.makedirs(save_path, exist_ok=True)

    # 打开bag
    storage_opts = StorageOptions(uri=bag_path, storage_id="sqlite3")
    conv_opts = ConverterOptions("", "")
    reader = SequentialReader()
    reader.open(storage_opts, conv_opts)

    # 准备CSV文件
    state_csv = os.path.join(save_path, "state.csv")
    imu_debug_csv = os.path.join(save_path, "imu_debug.csv")
    cov_csv = os.path.join(save_path, "covariance.csv")
    match_csv = os.path.join(save_path, "match_quality.csv")

    # CSV表头
    state_header = [
        "timestamp", "frame_time",
        "pos_x", "pos_y", "pos_z",
        "ori_x", "ori_y", "ori_z", "ori_w",
        "lin_x", "lin_y", "lin_z",
        "ang_x", "ang_y", "ang_z"
    ]

    imu_debug_header = [
        "timestamp", "frame_time",
        "imu_bias_acc_x", "imu_bias_acc_y", "imu_bias_acc_z",
        "imu_bias_gyro_x", "imu_bias_gyro_y", "imu_bias_gyro_z",
        "imu_preint_x", "imu_preint_y", "imu_preint_z",
        "velocity_x", "velocity_y", "velocity_z"
    ]

    cov_header = [
        "timestamp", "frame_time",
        "pos_cov_x", "pos_cov_y", "pos_cov_z",
        "ori_cov_x", "ori_cov_y", "ori_cov_z",
        "vel_cov_x", "vel_cov_y", "vel_cov_z"
    ]

    match_header = [
        "timestamp", "frame_time",
        "match_score", "match_ratio", "match_residual", "match_quality"
    ]

    # 数据缓存（用于合并同一frame_time的数据）
    odometry_cache = {}
    debug_cache = {}

    msg_count = 0

    print("正在读取bag数据...")
    while reader.has_next():
        topic, data, t = reader.read_next()

        if topic == "/odometry":
            msg = deserialize_message(data, Odometry)
            timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            frame_time = timestamp_to_frametime(timestamp)

            odometry_cache[frame_time] = {
                "timestamp": timestamp,
                "pose": msg.pose.pose,
                "pose_cov": list(msg.pose.covariance),
                "twist": msg.twist.twist,
                "twist_cov": list(msg.twist.covariance)
            }

        elif topic in ["/debug/imu_bias_acc", "/debug/imu_bias_gyro",
                       "/debug/imu_preint_pose", "/debug/velocity",
                       "/debug/match_quality"]:
            if topic == "/debug/imu_bias_acc":
                msg = deserialize_message(data, Vector3Stamped)
                timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                frame_time = timestamp_to_frametime(timestamp)
                if frame_time not in debug_cache:
                    debug_cache[frame_time] = {"timestamp": timestamp}
                debug_cache[frame_time]["imu_bias_acc"] = msg.vector

            elif topic == "/debug/imu_bias_gyro":
                msg = deserialize_message(data, Vector3Stamped)
                timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                frame_time = timestamp_to_frametime(timestamp)
                if frame_time not in debug_cache:
                    debug_cache[frame_time] = {"timestamp": timestamp}
                debug_cache[frame_time]["imu_bias_gyro"] = msg.vector

            elif topic == "/debug/imu_preint_pose":
                msg = deserialize_message(data, PoseStamped)
                timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                frame_time = timestamp_to_frametime(timestamp)
                if frame_time not in debug_cache:
                    debug_cache[frame_time] = {"timestamp": timestamp}
                debug_cache[frame_time]["imu_preint_pose"] = msg.pose

            elif topic == "/debug/velocity":
                msg = deserialize_message(data, Vector3Stamped)
                timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                frame_time = timestamp_to_frametime(timestamp)
                if frame_time not in debug_cache:
                    debug_cache[frame_time] = {"timestamp": timestamp}
                debug_cache[frame_time]["velocity"] = msg.vector

            elif topic == "/debug/match_quality":
                msg = deserialize_message(data, Float64MultiArray)
                timestamp = t * 1e-9
                frame_time = timestamp_to_frametime(timestamp)
                if frame_time not in debug_cache:
                    debug_cache[frame_time] = {"timestamp": timestamp}
                debug_cache[frame_time]["match_quality"] = list(msg.data)

        msg_count += 1
        if msg_count % 10000 == 0:
            print(f"  已读取 {msg_count} 条消息...")

    print(f"读取完成，共 {msg_count} 条消息")
    print(f"  odometry数据: {len(odometry_cache)} 帧")
    print(f"  debug数据: {len(debug_cache)} 帧")

    # 写入CSV文件
    print("\n正在写入CSV文件...")

    # 1. state.csv
    with open(state_csv, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(state_header)

        for frame_time in sorted(odometry_cache.keys()):
            data = odometry_cache[frame_time]
            p = data["pose"].position
            o = data["pose"].orientation
            lin = data["twist"].linear
            ang = data["twist"].angular

            writer.writerow([
                f"{data['timestamp']:.9f}",
                frame_time,
                p.x, p.y, p.z,
                o.x, o.y, o.z, o.w,
                lin.x, lin.y, lin.z,
                ang.x, ang.y, ang.z
            ])

    print(f"  state.csv: {len(odometry_cache)} 行")

    # 2. covariance.csv
    with open(cov_csv, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(cov_header)

        for frame_time in sorted(odometry_cache.keys()):
            data = odometry_cache[frame_time]
            pose_cov = data["pose_cov"]
            twist_cov = data["twist_cov"]

            # 提取协方差对角线元素
            # pose_cov: [x, y, z, rx, ry, rz] 的6x6矩阵
            # 对角线索引: 0, 7, 14, 21, 28, 35
            writer.writerow([
                f"{data['timestamp']:.9f}",
                frame_time,
                pose_cov[0], pose_cov[7], pose_cov[14],  # 位置方差
                pose_cov[21], pose_cov[28], pose_cov[35],  # 姿态方差
                twist_cov[0], twist_cov[7], twist_cov[14]  # 速度方差
            ])

    print(f"  covariance.csv: {len(odometry_cache)} 行")

    # 3. imu_debug.csv
    with open(imu_debug_csv, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(imu_debug_header)

        count_imu = 0
        for frame_time in sorted(debug_cache.keys()):
            data = debug_cache[frame_time]

            # 检查是否有完整的IMU数据
            if all(key in data for key in ["imu_bias_acc", "imu_bias_gyro", "velocity"]):
                acc_bias = data["imu_bias_acc"]
                gyro_bias = data["imu_bias_gyro"]
                velocity = data["velocity"]

                # IMU预积分位姿（可能不存在）
                preint_x = data["imu_preint_pose"].position.x if "imu_preint_pose" in data else 0.0
                preint_y = data["imu_preint_pose"].position.y if "imu_preint_pose" in data else 0.0
                preint_z = data["imu_preint_pose"].position.z if "imu_preint_pose" in data else 0.0

                writer.writerow([
                    f"{data['timestamp']:.9f}",
                    frame_time,
                    acc_bias.x, acc_bias.y, acc_bias.z,
                    gyro_bias.x, gyro_bias.y, gyro_bias.z,
                    preint_x, preint_y, preint_z,
                    velocity.x, velocity.y, velocity.z
                ])
                count_imu += 1

    print(f"  imu_debug.csv: {count_imu} 行")

    # 4. match_quality.csv
    with open(match_csv, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(match_header)

        count_match = 0
        for frame_time in sorted(debug_cache.keys()):
            data = debug_cache[frame_time]

            if "match_quality" in data and len(data["match_quality"]) >= 4:
                mq = data["match_quality"]
                writer.writerow([
                    f"{data['timestamp']:.9f}",
                    frame_time,
                    mq[0], mq[1], mq[2], mq[3]
                ])
                count_match += 1

    print(f"  match_quality.csv: {count_match} 行")

    print(f"\n完成! CSV文件已保存到: {save_path}")


def export_to_csv(save_path):
    """从JSON文件读取数据，导出为单个CSV文件

    输入:
        save_path: 包含JSON文件的根目录 (odometry/, state_other/ 等子目录)

    输出:
        analyse_data/analyse.csv: 包含所有关键数据
            - 时间戳、frame_time
            - 位置 (p_x, p_y, p_z)
            - 姿态四元数 (q_x, q_y, q_z, q_w) 和欧拉角 (roll, pitch, yaw)
            - 速度 (v_x, v_y, v_z) 和模长 (v_norm)
            - 角速度 (w_x, w_y, w_z) 和模长 (w_norm)
            - 位置协方差对角线 (pos_cov_x, pos_cov_y, pos_cov_z)
            - 姿态协方差对角线 (ori_cov_x, ori_cov_y, ori_cov_z)
            - IMU零偏 (bg_x, bg_y, bg_z, ba_x, ba_y, ba_z) 和模长 (bg_norm, ba_norm)
            - 外参 (offset_R_x/y/z, offset_T_x/y/z)
            - 重力 (grav_x, grav_y, grav_z)

    Args:
        save_path: JSON文件根目录路径
    """
    odometry_dir = os.path.join(save_path, "odometry")
    state_other_dir = os.path.join(save_path, "state_other")
    output_dir = os.path.join(save_path, "analyse_data")
    os.makedirs(output_dir, exist_ok=True)

    # 读取所有odometry数据
    print("正在读取odometry数据...")
    odometry_data = {}
    for filename in os.listdir(odometry_dir):
        if filename.endswith(".json"):
            filepath = os.path.join(odometry_dir, filename)
            with open(filepath, "r") as f:
                data = json.load(f)
                frame_time = data["frame_time"]
                odometry_data[frame_time] = data

    print(f"  读取到 {len(odometry_data)} 条odometry数据")

    # 读取所有state_other数据
    print("正在读取state_other数据...")
    state_other_data = {}
    if os.path.exists(state_other_dir):
        for filename in os.listdir(state_other_dir):
            if filename.endswith(".json"):
                filepath = os.path.join(state_other_dir, filename)
                with open(filepath, "r") as f:
                    data = json.load(f)
                    frame_time = data["frame_time"]
                    state_other_data[frame_time] = data

    print(f"  读取到 {len(state_other_data)} 条state_other数据")

    # 写入CSV
    output_csv = os.path.join(output_dir, "analyse.csv")

    # CSV表头
    header = [
        "timestamp", "frame_time",
        "p_x", "p_y", "p_z",
        "q_x", "q_y", "q_z", "q_w",
        "roll", "pitch", "yaw",
        "v_x", "v_y", "v_z", "v_norm",
        "w_x", "w_y", "w_z", "w_norm",
        "pos_cov_x", "pos_cov_y", "pos_cov_z",
        "ori_cov_x", "ori_cov_y", "ori_cov_z",
        "bg_x", "bg_y", "bg_z", "bg_norm",
        "ba_x", "ba_y", "ba_z", "ba_norm",
        "offset_R_x", "offset_R_y", "offset_R_z",
        "offset_T_x", "offset_T_y", "offset_T_z",
        "grav_x", "grav_y", "grav_z", "grav_norm"
    ]

    print("\n正在同步数据并写入CSV...")
    count_sync = 0
    count_odom_only = 0

    with open(output_csv, "w", newline="\n") as f:
        writer = csv.writer(f)
        writer.writerow(header)

        # 遍历所有odometry数据，尝试与state_other数据同步
        for frame_time in sorted(odometry_data.keys()):
            odom = odometry_data[frame_time]

            # 提取odometry数据
            timestamp = odom["timestamp"]
            p_x = odom["pose"]["position"]["x"]
            p_y = odom["pose"]["position"]["y"]
            p_z = odom["pose"]["position"]["z"]
            q_x = odom["pose"]["orientation"]["x"]
            q_y = odom["pose"]["orientation"]["y"]
            q_z = odom["pose"]["orientation"]["z"]
            q_w = odom["pose"]["orientation"]["w"]
            v_x = odom["twist"]["linear"]["x"]
            v_y = odom["twist"]["linear"]["y"]
            v_z = odom["twist"]["linear"]["z"]
            w_x = odom["twist"]["angular"]["x"]
            w_y = odom["twist"]["angular"]["y"]
            w_z = odom["twist"]["angular"]["z"]

            # 四元数转欧拉角 (roll, pitch, yaw)
            roll = np.arctan2(2.0 * (q_w * q_x + q_y * q_z), 1.0 - 2.0 * (q_x * q_x + q_y * q_y))
            pitch = np.arcsin(np.clip(2.0 * (q_w * q_y - q_z * q_x), -1.0, 1.0))
            yaw = np.arctan2(2.0 * (q_w * q_z + q_x * q_y), 1.0 - 2.0 * (q_y * q_y + q_z * q_z))

            # 计算速度和角速度模长
            v_norm = np.sqrt(v_x**2 + v_y**2 + v_z**2)
            w_norm = np.sqrt(w_x**2 + w_y**2 + w_z**2)

            # 提取协方差对角线
            pose_cov = odom["pose"]["covariance"]
            pos_cov_x = pose_cov[0]  # x方差
            pos_cov_y = pose_cov[7]  # y方差
            pos_cov_z = pose_cov[14]  # z方差
            ori_cov_x = pose_cov[21]  # rx方差
            ori_cov_y = pose_cov[28]  # ry方差
            ori_cov_z = pose_cov[35]  # rz方差

            # 尝试从state_other数据中获取IMU零偏、外参、重力
            bg_x, bg_y, bg_z = 0.0, 0.0, 0.0
            ba_x, ba_y, ba_z = 0.0, 0.0, 0.0
            offset_R_x, offset_R_y, offset_R_z = 0.0, 0.0, 0.0
            offset_T_x, offset_T_y, offset_T_z = 0.0, 0.0, 0.0
            grav_x, grav_y, grav_z = 0.0, 0.0, 0.0

            if frame_time in state_other_data:
                other = state_other_data[frame_time]
                bg_x = other["bg"]["x"]
                bg_y = other["bg"]["y"]
                bg_z = other["bg"]["z"]
                ba_x = other["ba"]["x"]
                ba_y = other["ba"]["y"]
                ba_z = other["ba"]["z"]
                offset_R_x = other["offset_R_L_I"]["x"]
                offset_R_y = other["offset_R_L_I"]["y"]
                offset_R_z = other["offset_R_L_I"]["z"]
                offset_T_x = other["offset_T_L_I"]["x"]
                offset_T_y = other["offset_T_L_I"]["y"]
                offset_T_z = other["offset_T_L_I"]["z"]
                grav_x = other["grav"]["x"]
                grav_y = other["grav"]["y"]
                grav_z = other["grav"]["z"]
                count_sync += 1
            else:
                count_odom_only += 1

            # 计算零偏模长
            bg_norm = np.sqrt(bg_x**2 + bg_y**2 + bg_z**2)
            ba_norm = np.sqrt(ba_x**2 + ba_y**2 + ba_z**2)
            grav_norm = np.sqrt(grav_x**2 + grav_y**2 + grav_z**2)

            # 写入CSV行 - 使用固定小数格式，PlotJuggler兼容
            def fmt(val):
                """格式化数值为固定小数格式"""
                return f"{val:.12f}"

            writer.writerow([
                f"{timestamp:.9f}",
                frame_time,
                fmt(p_x), fmt(p_y), fmt(p_z),
                fmt(q_x), fmt(q_y), fmt(q_z), fmt(q_w),
                fmt(roll), fmt(pitch), fmt(yaw),
                fmt(v_x), fmt(v_y), fmt(v_z), fmt(v_norm),
                fmt(w_x), fmt(w_y), fmt(w_z), fmt(w_norm),
                fmt(pos_cov_x), fmt(pos_cov_y), fmt(pos_cov_z),
                fmt(ori_cov_x), fmt(ori_cov_y), fmt(ori_cov_z),
                fmt(bg_x), fmt(bg_y), fmt(bg_z), fmt(bg_norm),
                fmt(ba_x), fmt(ba_y), fmt(ba_z), fmt(ba_norm),
                fmt(offset_R_x), fmt(offset_R_y), fmt(offset_R_z),
                fmt(offset_T_x), fmt(offset_T_y), fmt(offset_T_z),
                fmt(grav_x), fmt(grav_y), fmt(grav_z), fmt(grav_norm)
            ])

    print(f"  成功同步: {count_sync} 帧")
    print(f"  仅odometry: {count_odom_only} 帧")
    print(f"\n完成! CSV文件已保存到: {output_csv}")


if __name__ == "__main__":
    bag_path = "/home/jerett/OpenProject/LidarSlam/spark-fast-lio/spark_fast_lio/scripts/data/lio_20260429_200150"
    save_path = "/home/jerett/OpenProject/LidarSlam/spark-fast-lio/spark_fast_lio/scripts/data/lio_20260429_200150/asset_data"

    # 第一步：解析bag文件
    print("=" * 60)
    print("Step 1: 解析bag文件")
    print("=" * 60)
    parse_all_topics(bag_path, save_path)

    # 第二步：导出CSV
    print("\n" + "=" * 60)
    print("Step 2: 导出CSV")
    print("=" * 60)
    export_to_csv(save_path)
