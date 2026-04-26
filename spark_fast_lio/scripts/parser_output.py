#!/usr/bin/env python3
"""
Output数据解析脚本
将output目录中的状态数据整理成CSV文件，用于PlotJuggler分析

使用方法:
    python3 parser_output.py <asset_data> <output>

参数说明:
    asset_data: 数据同步后的文件夹地址 (包含frame_info, imu等子目录)
    output:     滤波后的文件夹地址 (包含pred_state, update_state等子目录)

输出:
    CSV文件保存至 {output}/analyse_{时间}.csv

输出CSV列:
    - 时间戳: frame_time, lidar_begin, lidar_end, imu_begin, imu_end
    - 预测状态: pred_x/y/z, pred_roll/pitch/yaw, pred_bg_x/y/z, pred_ba_x/y/z, pred_Ril_x/y/z/w, pred_Til_x/y/z
    - 更新状态: update_x/y/z, update_roll/pitch/yaw, update_bg_x/y/z, update_ba_x/y/z, update_Ril_x/y/z/w, update_Til_x/y/z
"""

import csv
import os
import sys
import math
from datetime import datetime


def parse_frame_time(frame_time_str):
    """
    解析frame_time字符串为秒数
    格式: "SSSSSSSSSSSS_PPPPPPPPPPPP" (12位秒数_12位皮秒数)
    """
    parts = frame_time_str.split('_')
    if len(parts) != 2:
        return 0.0
    seconds = int(parts[0])
    picoseconds = int(parts[1])
    return seconds + picoseconds * 1e-12


def load_state_csv(csv_path):
    """加载状态CSV文件，返回字典"""
    if not os.path.exists(csv_path):
        return None
    with open(csv_path, 'r') as f:
        line = f.readline().strip()
        if not line:
            return None
        values = line.split(',')
        if len(values) < 28:
            return None
        # CSV格式: frame_time,pos_x,pos_y,pos_z,quat_w,quat_x,quat_y,quat_z,
        #         euler_roll,euler_pitch,euler_yaw,vel_x,vel_y,vel_z,
        #         bg_x,bg_y,bg_z,ba_x,ba_y,ba_z,grav_x,grav_y,grav_z,
        #         Ril_w,Ril_x,Ril_y,Ril_z,Til_x,Til_y,Til_z
        return {
            'frame_time': values[0],
            'position': [float(values[1]), float(values[2]), float(values[3])],
            'orientation_quat': [float(values[4]), float(values[5]), float(values[6]), float(values[7])],
            'orientation_euler_deg': [float(values[8]), float(values[9]), float(values[10])],
            'velocity': [float(values[11]), float(values[12]), float(values[13])],
            'bias_gyro': [float(values[14]), float(values[15]), float(values[16])],
            'bias_acc': [float(values[17]), float(values[18]), float(values[19])],
            'gravity': [float(values[20]), float(values[21]), float(values[22])],
            'offset_R_L_I': [float(values[23]), float(values[24]), float(values[25]), float(values[26])],
            'offset_T_L_I': [float(values[27]), float(values[28]), float(values[29])] if len(values) > 28 else [0, 0, 0]
        }


def load_frame_info(frame_info_path):
    """加载帧信息JSON文件"""
    import json
    if not os.path.exists(frame_info_path):
        return None
    with open(frame_info_path, 'r') as f:
        return json.load(f)


def get_imu_time_range(imu_csv_path):
    """从IMU CSV文件获取时间范围"""
    if not os.path.exists(imu_csv_path):
        return None, None

    imu_begin = None
    imu_end = None

    with open(imu_csv_path, 'r') as f:
        reader = csv.reader(f)
        header = next(reader, None)  # Skip header
        if header is None:
            return None, None

        for row in reader:
            if len(row) > 0:
                try:
                    timestamp = float(row[0])
                    if imu_begin is None:
                        imu_begin = timestamp
                    imu_end = timestamp
                except ValueError:
                    continue

    return imu_begin, imu_end


def parse_output_data(output_path, asset_data_path):
    """
    解析output目录中的所有数据

    返回: 按frame_time排序的数据列表
    """
    data_records = {}

    # 获取所有frame_time (从pred_state目录)
    pred_state_dir = os.path.join(output_path, 'pred_state')
    if not os.path.exists(pred_state_dir):
        print(f"Error: pred_state directory not found: {pred_state_dir}")
        return []

    # 遍历所有CSV文件
    for filename in os.listdir(pred_state_dir):
        if not filename.endswith('.csv'):
            continue

        frame_time = filename[:-4]  # Remove .csv
        data_records[frame_time] = {'frame_time': frame_time}

    print(f"Found {len(data_records)} frames in pred_state")

    # 加载预测状态
    for frame_time in data_records:
        csv_path = os.path.join(output_path, 'pred_state', f'{frame_time}.csv')
        state = load_state_csv(csv_path)
        if state:
            data_records[frame_time]['pred_state'] = state

    # 加载更新状态
    for frame_time in data_records:
        csv_path = os.path.join(output_path, 'update_state', f'{frame_time}.csv')
        state = load_state_csv(csv_path)
        if state:
            data_records[frame_time]['update_state'] = state

    # 加载帧信息 (lidar_begin, lidar_end)
    if asset_data_path:
        frame_info_dir = os.path.join(asset_data_path, 'frame_info')
        imu_dir = os.path.join(asset_data_path, 'imu')

        for frame_time in data_records:
            # 帧信息
            frame_info_path = os.path.join(frame_info_dir, f'{frame_time}.json')
            frame_info = load_frame_info(frame_info_path)
            if frame_info:
                data_records[frame_time]['lidar_begin'] = frame_info.get('begin_time')
                data_records[frame_time]['lidar_end'] = frame_info.get('end_time')

            # IMU时间范围
            imu_csv_path = os.path.join(imu_dir, f'{frame_time}.csv')
            imu_begin, imu_end = get_imu_time_range(imu_csv_path)
            if imu_begin is not None:
                data_records[frame_time]['imu_begin'] = imu_begin
            if imu_end is not None:
                data_records[frame_time]['imu_end'] = imu_end

    # 按frame_time排序
    sorted_times = sorted(data_records.keys(), key=lambda x: parse_frame_time(x))

    return [data_records[t] for t in sorted_times]


def write_csv(data_records, csv_path):
    """将数据写入CSV文件"""
    # CSV列定义
    header = [
        # 时间戳
        'frame_time', 'frame_time_sec',
        'lidar_begin', 'lidar_end', 'imu_begin', 'imu_end',
        # 预测状态 - 位置
        'pred_x', 'pred_y', 'pred_z',
        # 预测状态 - 姿态 (欧拉角)
        'pred_roll', 'pred_pitch', 'pred_yaw',
        # 预测状态 - 陀螺仪bias
        'pred_bg_x', 'pred_bg_y', 'pred_bg_z',
        # 预测状态 - 加速度计bias
        'pred_ba_x', 'pred_ba_y', 'pred_ba_z',
        # 预测状态 - LiDAR-IMU外参旋转 (四元数)
        'pred_Ril_w', 'pred_Ril_x', 'pred_Ril_y', 'pred_Ril_z',
        # 预测状态 - LiDAR-IMU外参平移
        'pred_Til_x', 'pred_Til_y', 'pred_Til_z',
        # 更新状态 - 位置
        'update_x', 'update_y', 'update_z',
        # 更新状态 - 姿态 (欧拉角)
        'update_roll', 'update_pitch', 'update_yaw',
        # 更新状态 - 陀螺仪bias
        'update_bg_x', 'update_bg_y', 'update_bg_z',
        # 更新状态 - 加速度计bias
        'update_ba_x', 'update_ba_y', 'update_ba_z',
        # 更新状态 - LiDAR-IMU外参旋转 (四元数)
        'update_Ril_w', 'update_Ril_x', 'update_Ril_y', 'update_Ril_z',
        # 更新状态 - LiDAR-IMU外参平移
        'update_Til_x', 'update_Til_y', 'update_Til_z',
    ]

    with open(csv_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(header)

        for record in data_records:
            row = []

            # 时间戳
            frame_time = record.get('frame_time', '')
            row.append(frame_time)
            row.append(f'{parse_frame_time(frame_time):.6f}')

            # lidar_begin, lidar_end
            lidar_begin = record.get('lidar_begin')
            row.append(f'{lidar_begin:.9f}' if lidar_begin is not None else '')
            lidar_end = record.get('lidar_end')
            row.append(f'{lidar_end:.9f}' if lidar_end is not None else '')

            # imu_begin, imu_end
            imu_begin = record.get('imu_begin')
            row.append(f'{imu_begin:.9f}' if imu_begin is not None else '')
            imu_end = record.get('imu_end')
            row.append(f'{imu_end:.9f}' if imu_end is not None else '')

            # 预测状态
            pred_state = record.get('pred_state')
            if pred_state:
                # 位置
                pos = pred_state.get('position', [0, 0, 0])
                row.extend([f'{pos[0]:.9f}', f'{pos[1]:.9f}', f'{pos[2]:.9f}'])

                # 姿态 - 直接使用euler_deg (已经是度数，需要转换为弧度)
                euler_deg = pred_state.get('orientation_euler_deg', [0, 0, 0])
                roll = euler_deg[0] * math.pi / 180.0
                pitch = euler_deg[1] * math.pi / 180.0
                yaw = euler_deg[2] * math.pi / 180.0
                row.extend([f'{roll:.9f}', f'{pitch:.9f}', f'{yaw:.9f}'])

                # 陀螺仪bias
                bg = pred_state.get('bias_gyro', [0, 0, 0])
                row.extend([f'{bg[0]:.9f}', f'{bg[1]:.9f}', f'{bg[2]:.9f}'])

                # 加速度计bias
                ba = pred_state.get('bias_acc', [0, 0, 0])
                row.extend([f'{ba[0]:.9f}', f'{ba[1]:.9f}', f'{ba[2]:.9f}'])

                # LiDAR-IMU外参旋转 (四元数)
                Ril = pred_state.get('offset_R_L_I', [1, 0, 0, 0])  # [w, x, y, z]
                row.extend([f'{Ril[0]:.9f}', f'{Ril[1]:.9f}', f'{Ril[2]:.9f}', f'{Ril[3]:.9f}'])

                # LiDAR-IMU外参平移
                Til = pred_state.get('offset_T_L_I', [0, 0, 0])
                row.extend([f'{Til[0]:.9f}', f'{Til[1]:.9f}', f'{Til[2]:.9f}'])
            else:
                row.extend([''] * 16)

            # 更新状态
            update_state = record.get('update_state')
            if update_state:
                # 位置
                pos = update_state.get('position', [0, 0, 0])
                row.extend([f'{pos[0]:.9f}', f'{pos[1]:.9f}', f'{pos[2]:.9f}'])

                # 姿态 - 直接使用euler_deg
                euler_deg = update_state.get('orientation_euler_deg', [0, 0, 0])
                roll = euler_deg[0] * math.pi / 180.0
                pitch = euler_deg[1] * math.pi / 180.0
                yaw = euler_deg[2] * math.pi / 180.0
                row.extend([f'{roll:.9f}', f'{pitch:.9f}', f'{yaw:.9f}'])

                # 陀螺仪bias
                bg = update_state.get('bias_gyro', [0, 0, 0])
                row.extend([f'{bg[0]:.9f}', f'{bg[1]:.9f}', f'{bg[2]:.9f}'])

                # 加速度计bias
                ba = update_state.get('bias_acc', [0, 0, 0])
                row.extend([f'{ba[0]:.9f}', f'{ba[1]:.9f}', f'{ba[2]:.9f}'])

                # LiDAR-IMU外参旋转 (四元数)
                Ril = update_state.get('offset_R_L_I', [1, 0, 0, 0])  # [w, x, y, z]
                row.extend([f'{Ril[0]:.9f}', f'{Ril[1]:.9f}', f'{Ril[2]:.9f}', f'{Ril[3]:.9f}'])

                # LiDAR-IMU外参平移
                Til = update_state.get('offset_T_L_I', [0, 0, 0])
                row.extend([f'{Til[0]:.9f}', f'{Til[1]:.9f}', f'{Til[2]:.9f}'])
            else:
                row.extend([''] * 16)

            writer.writerow(row)

    print(f"CSV written: {csv_path}")
    print(f"Total rows: {len(data_records)}")


def main():
    if len(sys.argv) < 3:
        print("Usage: python3 parser_output.py <asset_data> <output>")
        print("")
        print("Arguments:")
        print("  asset_data: Path containing frame_info, imu directories")
        print("  output:     Path containing pred_state, update_state directories")
        print("")
        print("Output:")
        print("  CSV file saved to {output}/analyse_{timestamp}.csv")
        print("")
        print("Example:")
        print("  python3 parser_output.py \\")
        print("    /path/to/asset_data \\")
        print("    /path/to/output")
        sys.exit(1)

    asset_data_path = sys.argv[1]
    output_path = sys.argv[2]

    # 验证路径
    if not os.path.exists(output_path):
        print(f"Error: output path does not exist: {output_path}")
        sys.exit(1)

    if not os.path.exists(asset_data_path):
        print(f"Warning: asset_data path does not exist: {asset_data_path}")
        asset_data_path = None

    # 生成输出CSV路径
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    analyse_path = os.path.join(output_path, f'analyse_{timestamp}.csv')

    print(f"Asset data: {asset_data_path if asset_data_path else 'not found'}")
    print(f"Output: {output_path}")
    print(f"Analyse: {analyse_path}")
    print("")

    # 解析数据
    data_records = parse_output_data(output_path, asset_data_path)

    if not data_records:
        print("Error: No data records found")
        sys.exit(1)

    # 写入CSV
    write_csv(data_records, analyse_path)


if __name__ == '__main__':
    main()