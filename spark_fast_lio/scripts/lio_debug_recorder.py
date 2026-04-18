#!/usr/bin/env python3
"""
LIO调试数据记录脚本
订阅调试topic并保存成CSV文件，用于PlotJuggler离线分析

使用方法:
    python3 lio_debug_recorder.py

输出目录:
    logs/<timestamp>/debug_data/
        - lio_debug_pose.csv: 最终Pose和IMU预积分Pose
        - lio_debug_delta.csv: 帧间Delta Pose
        - lio_debug_bias.csv: IMU Bias
        - lio_debug_quality.csv: 匹配质量
        - lio_debug_velocity.csv: 速度

PlotJuggler加载CSV:
    File -> Load Data -> 选择CSV文件
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Pose, Vector3Stamped
from std_msgs.msg import Float64MultiArray

import csv
import os
import sys
from datetime import datetime
from collections import defaultdict

# 项目根目录
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


class LIODebugRecorder(Node):
    def __init__(self):
        super().__init__('lio_debug_recorder')

        # 创建带时间戳的输出目录
        self.timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.log_dir = os.path.join(PROJECT_ROOT, 'logs', self.timestamp)
        self.output_dir = os.path.join(self.log_dir, 'debug_data')
        os.makedirs(self.output_dir, exist_ok=True)

        # CSV文件和writer
        self.csv_files = {}
        self.csv_writers = {}

        # 初始化各CSV文件
        self._init_csv('pose', [
            'timestamp',
            'odom_x', 'odom_y', 'odom_z',
            'odom_qx', 'odom_qy', 'odom_qz', 'odom_qw',
            'odom_roll', 'odom_pitch', 'odom_yaw',
            'preint_x', 'preint_y', 'preint_z',
            'preint_qx', 'preint_qy', 'preint_qz', 'preint_qw',
            'preint_roll', 'preint_pitch', 'preint_yaw',
        ])

        self._init_csv('delta', [
            'timestamp',
            'delta_x', 'delta_y', 'delta_z',
            'delta_roll', 'delta_pitch', 'delta_yaw',
        ])

        self._init_csv('bias', [
            'timestamp',
            'bg_x', 'bg_y', 'bg_z',
            'ba_x', 'ba_y', 'ba_z',
        ])

        self._init_csv('quality', [
            'timestamp',
            'feat_num', 'res_mean', 'solve_time',
        ])

        self._init_csv('velocity', [
            'timestamp',
            'vx', 'vy', 'vz',
        ])

        # QoS设置
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        # 数据缓存（用于同步不同topic的数据）
        self.data_cache = defaultdict(dict)

        # 订阅topics
        self.sub_odom = self.create_subscription(
            Odometry, '/odometry', self.cb_odom, qos)

        self.sub_preint = self.create_subscription(
            PoseStamped, '/debug/imu_preint_pose', self.cb_preint, qos)

        self.sub_delta = self.create_subscription(
            Pose, '/debug/delta_pose', self.cb_delta, qos)

        self.sub_bias_gyro = self.create_subscription(
            Vector3Stamped, '/debug/imu_bias_gyro', self.cb_bias_gyro, qos)

        self.sub_bias_acc = self.create_subscription(
            Vector3Stamped, '/debug/imu_bias_acc', self.cb_bias_acc, qos)

        self.sub_quality = self.create_subscription(
            Float64MultiArray, '/debug/match_quality', self.cb_quality, qos)

        self.sub_velocity = self.create_subscription(
            Vector3Stamped, '/debug/velocity', self.cb_velocity, qos)

        self.get_logger().info(f'LIO Debug Recorder started')
        self.get_logger().info(f'Log directory: {self.log_dir}')

        # 写入运行信息
        self._write_run_info()

    def _write_run_info(self):
        """写入运行信息"""
        info_file = os.path.join(self.log_dir, 'run_info.txt')
        with open(info_file, 'w') as f:
            f.write(f'Run Time: {self.timestamp}\n')
            f.write(f'Output Directory: {self.output_dir}\n')
            f.write(f'\nSubscribed Topics:\n')
            f.write(f'  - /odometry\n')
            f.write(f'  - /debug/imu_preint_pose\n')
            f.write(f'  - /debug/delta_pose\n')
            f.write(f'  - /debug/imu_bias_gyro\n')
            f.write(f'  - /debug/imu_bias_acc\n')
            f.write(f'  - /debug/match_quality\n')
            f.write(f'  - /debug/velocity\n')

    def _init_csv(self, name, headers):
        """初始化CSV文件"""
        filepath = os.path.join(self.output_dir, f'lio_debug_{name}.csv')
        self.csv_files[name] = open(filepath, 'w', newline='')
        self.csv_writers[name] = csv.writer(self.csv_files[name])
        self.csv_writers[name].writerow(headers)

    def _quat_to_rpy(self, qx, qy, qz, qw):
        """四元数转欧拉角"""
        import math
        # Roll
        sinr_cosp = 2 * (qw * qx + qy * qz)
        cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch
        sinp = 2 * (qw * qy - qz * qx)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        # Yaw
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def cb_odom(self, msg: Odometry):
        """最终Pose回调"""
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        roll, pitch, yaw = self._quat_to_rpy(o.x, o.y, o.z, o.w)

        self.data_cache[t]['odom'] = [
            p.x, p.y, p.z,
            o.x, o.y, o.z, o.w,
            roll, pitch, yaw,
        ]
        self._try_write_pose(t)

    def cb_preint(self, msg: PoseStamped):
        """IMU预积分Pose回调"""
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.position
        o = msg.pose.orientation
        roll, pitch, yaw = self._quat_to_rpy(o.x, o.y, o.z, o.w)

        self.data_cache[t]['preint'] = [
            p.x, p.y, p.z,
            o.x, o.y, o.z, o.w,
            roll, pitch, yaw,
        ]
        self._try_write_pose(t)

    def _try_write_pose(self, t):
        """尝试写入pose数据（需要odom和preint都到达）"""
        if 'odom' in self.data_cache[t] and 'preint' in self.data_cache[t]:
            row = [t] + self.data_cache[t]['odom'] + self.data_cache[t]['preint']
            self.csv_writers['pose'].writerow(row)
            del self.data_cache[t]

    def cb_delta(self, msg: Pose):
        """帧间Delta Pose回调"""
        # 使用当前时间戳
        t = self.get_clock().now().nanoseconds * 1e-9
        p = msg.position
        o = msg.orientation
        roll, pitch, yaw = self._quat_to_rpy(o.x, o.y, o.z, o.w)

        self.csv_writers['delta'].writerow([
            t, p.x, p.y, p.z, roll, pitch, yaw
        ])

    def cb_bias_gyro(self, msg: Vector3Stamped):
        """陀螺仪Bias回调"""
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.data_cache[t]['bg'] = [msg.vector.x, msg.vector.y, msg.vector.z]
        self._try_write_bias(t)

    def cb_bias_acc(self, msg: Vector3Stamped):
        """加速度计Bias回调"""
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.data_cache[t]['ba'] = [msg.vector.x, msg.vector.y, msg.vector.z]
        self._try_write_bias(t)

    def _try_write_bias(self, t):
        """尝试写入bias数据"""
        if 'bg' in self.data_cache[t] and 'ba' in self.data_cache[t]:
            row = [t] + self.data_cache[t]['bg'] + self.data_cache[t]['ba']
            self.csv_writers['bias'].writerow(row)
            del self.data_cache[t]

    def cb_quality(self, msg: Float64MultiArray):
        """匹配质量回调"""
        t = self.get_clock().now().nanoseconds * 1e-9
        if len(msg.data) >= 3:
            self.csv_writers['quality'].writerow([
                t, msg.data[0], msg.data[1], msg.data[2]
            ])

    def cb_velocity(self, msg: Vector3Stamped):
        """速度回调"""
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.csv_writers['velocity'].writerow([
            t, msg.vector.x, msg.vector.y, msg.vector.z
        ])

    def close(self):
        """关闭所有CSV文件"""
        for f in self.csv_files.values():
            f.close()
        self.get_logger().info(f'CSV files saved to: {self.output_dir}')


def main(args=None):
    rclpy.init(args=args)
    node = LIODebugRecorder()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
