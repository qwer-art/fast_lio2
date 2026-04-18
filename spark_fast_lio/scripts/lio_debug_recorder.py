#!/usr/bin/env python3
"""
LIO调试数据记录脚本
订阅调试topic并保存成CSV文件，用于PlotJuggler离线分析

使用方法:
    python3 lio_debug_recorder.py

输出文件:
    logs/<timestamp>/lio_debug.csv - 所有数据合并到一个文件，按frame_time对齐

PlotJuggler加载CSV:
    File -> Load Data -> 选择 lio_debug.csv
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Pose, Vector3Stamped
from std_msgs.msg import Float64MultiArray

import csv
import os
from datetime import datetime

# 项目根目录
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


class LIODebugRecorder(Node):
    def __init__(self):
        super().__init__('lio_debug_recorder')

        # 创建带时间戳的输出目录
        self.timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.log_dir = os.path.join(PROJECT_ROOT, 'logs', self.timestamp)
        os.makedirs(self.log_dir, exist_ok=True)

        # 最新数据缓存（用于无时间戳的消息）
        self.latest_quality = None  # [feat_num, res_mean, solve_time]
        self.latest_delta = None    # [x, y, z, roll, pitch, yaw]

        # 上一次写入的时间戳（用于检查单调性）
        self.last_timestamp = 0.0

        # CSV文件
        self.csv_path = os.path.join(self.log_dir, 'lio_debug.csv')
        self.csv_file = open(self.csv_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)

        # 写入header
        self.csv_writer.writerow([
            'timestamp', 'frame_time',
            # Pose数据
            'odom_x', 'odom_y', 'odom_z',
            'odom_roll', 'odom_pitch', 'odom_yaw',
            'preint_x', 'preint_y', 'preint_z',
            'preint_roll', 'preint_pitch', 'preint_yaw',
            # IMU Bias
            'bg_x', 'bg_y', 'bg_z',
            'ba_x', 'ba_y', 'ba_z',
            # 速度
            'vx', 'vy', 'vz',
            # 匹配质量
            'feat_num', 'lidar_res', 'solve_time', 'imu_res',
            # Delta Pose
            'delta_x', 'delta_y', 'delta_z',
            'delta_roll', 'delta_pitch', 'delta_yaw',
        ])

        # QoS设置
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        # 订阅topics
        self.create_subscription(Odometry, '/odometry', self.cb_odom, qos)
        self.create_subscription(PoseStamped, '/debug/imu_preint_pose', self.cb_preint, qos)
        self.create_subscription(Pose, '/debug/delta_pose', self.cb_delta, qos)
        self.create_subscription(Vector3Stamped, '/debug/imu_bias_gyro', self.cb_bias_gyro, qos)
        self.create_subscription(Vector3Stamped, '/debug/imu_bias_acc', self.cb_bias_acc, qos)
        self.create_subscription(Float64MultiArray, '/debug/match_quality', self.cb_quality, qos)
        self.create_subscription(Vector3Stamped, '/debug/velocity', self.cb_velocity, qos)

        # 当前帧数据缓存
        self.current_frame = {}

        self.get_logger().info(f'LIO Debug Recorder started')
        self.get_logger().info(f'Output: {self.csv_path}')

    def _to_frame_time(self, timestamp):
        """将时间戳对齐到50ms网格"""
        return round(timestamp / 0.05) * 0.05

    def _quat_to_rpy(self, qx, qy, qz, qw):
        """四元数转欧拉角"""
        import math
        sinr_cosp = 2 * (qw * qx + qy * qz)
        cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (qw * qy - qz * qx)
        pitch = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1 else math.asin(sinp)

        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def _write_row(self, timestamp, odom, preint=None, bg=None, ba=None, vel=None, quality=None, delta=None):
        """写入一行数据"""
        frame_time = self._to_frame_time(timestamp)

        row = [f'{timestamp:.6f}', f'{frame_time:.3f}']

        # odom pose
        if odom:
            row.extend([f'{odom[0]:.6f}', f'{odom[1]:.6f}', f'{odom[2]:.6f}',
                        f'{odom[3]:.6f}', f'{odom[4]:.6f}', f'{odom[5]:.6f}'])
        else:
            row.extend([''] * 6)

        # preint pose
        if preint:
            row.extend([f'{preint[0]:.6f}', f'{preint[1]:.6f}', f'{preint[2]:.6f}',
                        f'{preint[3]:.6f}', f'{preint[4]:.6f}', f'{preint[5]:.6f}'])
        else:
            row.extend([''] * 6)

        # gyro bias
        if bg:
            row.extend([f'{bg[0]:.6f}', f'{bg[1]:.6f}', f'{bg[2]:.6f}'])
        else:
            row.extend([''] * 3)

        # acc bias
        if ba:
            row.extend([f'{ba[0]:.6f}', f'{ba[1]:.6f}', f'{ba[2]:.6f}'])
        else:
            row.extend([''] * 3)

        # velocity
        if vel:
            row.extend([f'{vel[0]:.6f}', f'{vel[1]:.6f}', f'{vel[2]:.6f}'])
        else:
            row.extend([''] * 3)

        # quality (feat_num, lidar_res, solve_time, imu_res)
        if quality:
            row.extend([f'{quality[0]:.0f}', f'{quality[1]:.6f}', f'{quality[2]:.6f}', f'{quality[3]:.6f}'])
        else:
            row.extend([''] * 4)

        # delta pose
        if delta:
            row.extend([f'{delta[0]:.6f}', f'{delta[1]:.6f}', f'{delta[2]:.6f}',
                        f'{delta[3]:.6f}', f'{delta[4]:.6f}', f'{delta[5]:.6f}'])
        else:
            row.extend([''] * 6)

        self.csv_writer.writerow(row)
        self.csv_file.flush()

    def cb_odom(self, msg: Odometry):
        """odom作为主触发器，写入一行数据"""
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        # 检查时间戳单调性，跳过乱序消息
        if t < self.last_timestamp:
            return
        self.last_timestamp = t

        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        roll, pitch, yaw = self._quat_to_rpy(o.x, o.y, o.z, o.w)
        odom = [p.x, p.y, p.z, roll, pitch, yaw]

        # 获取当前帧的其他数据
        preint = self.current_frame.get('preint')
        bg = self.current_frame.get('bg')
        ba = self.current_frame.get('ba')
        vel = self.current_frame.get('vel')

        # 使用最新的quality和delta（这些消息没有时间戳）
        quality = self.latest_quality
        delta = self.latest_delta

        self._write_row(t, odom, preint, bg, ba, vel, quality, delta)

        # 清空当前帧缓存
        self.current_frame = {}

    def cb_preint(self, msg: PoseStamped):
        """IMU预积分Pose"""
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.position
        o = msg.pose.orientation
        roll, pitch, yaw = self._quat_to_rpy(o.x, o.y, o.z, o.w)
        self.current_frame['preint'] = [p.x, p.y, p.z, roll, pitch, yaw]

    def cb_delta(self, msg: Pose):
        """帧间Delta Pose - 无时间戳，缓存最新值"""
        p = msg.position
        o = msg.orientation
        roll, pitch, yaw = self._quat_to_rpy(o.x, o.y, o.z, o.w)
        self.latest_delta = [p.x, p.y, p.z, roll, pitch, yaw]

    def cb_bias_gyro(self, msg: Vector3Stamped):
        """陀螺仪Bias"""
        self.current_frame['bg'] = [msg.vector.x, msg.vector.y, msg.vector.z]

    def cb_bias_acc(self, msg: Vector3Stamped):
        """加速度计Bias"""
        self.current_frame['ba'] = [msg.vector.x, msg.vector.y, msg.vector.z]

    def cb_quality(self, msg: Float64MultiArray):
        """匹配质量 - 无时间戳，缓存最新值"""
        if len(msg.data) >= 4:
            self.latest_quality = [msg.data[0], msg.data[1], msg.data[2], msg.data[3]]
        elif len(msg.data) >= 3:
            # 兼容旧版本（没有imu_res）
            self.latest_quality = [msg.data[0], msg.data[1], msg.data[2], 0.0]

    def cb_velocity(self, msg: Vector3Stamped):
        """速度"""
        self.current_frame['vel'] = [msg.vector.x, msg.vector.y, msg.vector.z]

    def close(self):
        self.csv_file.close()
        print(f'CSV saved: {self.csv_path}')


def main(args=None):
    rclpy.init(args=args)
    node = LIODebugRecorder()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
