"""
FastLIO2 Visualization Utilities
"""

import numpy as np
import cv2
import OpenGL.GL as gl
import pangolin
import json
import glob
import os
import re
from natsort import natsorted


class Color:
    """Color definitions in BGR format"""
    kRed = (0, 0, 255)
    kGreen = (0, 255, 0)
    kBlue = (255, 0, 0)
    kYellow = (0, 255, 255)
    kMagenta = (255, 0, 255)
    kCyan = (255, 255, 0)
    kWhite = (255, 255, 255)
    kBlack = (0, 0, 0)
    kOrange = (0, 165, 255)
    kGray = (128, 128, 128)

    # Dark colors for better readability
    kDarkRed = (0, 0, 180)
    kDarkGreen = (0, 150, 0)
    kDarkBlue = (180, 0, 0)
    kDarkCyan = (180, 180, 0)
    kDarkMagenta = (180, 0, 180)
    kDarkOrange = (0, 120, 200)
    kDarkYellow = (0, 180, 180)
    kDarkGray = (100, 100, 100)


def color2bgr(color):
    """Convert color to BGR format"""
    return color


def set_gl_color(color_type):
    """Set OpenGL color from Color enum"""
    bgr = color2bgr(color_type)
    gl.glColor3f(bgr[2]/255.0, bgr[1]/255.0, bgr[0]/255.0)


def transform_points(pose, points):
    """Transform points using pose matrix"""
    original_shape = points.shape
    points_reshaped = points.reshape(-1, 3)
    R = pose[:3, :3]
    points_r = np.einsum('ij,mj->mi', R, points_reshaped)
    t = pose[:3, 3]
    points_tf = points_r + t
    return points_tf.reshape(original_shape)


def quaternion_to_rotation_matrix(qx, qy, qz, qw):
    """Convert quaternion to rotation matrix"""
    xx, yy, zz = qx * qx, qy * qy, qz * qz
    xy, xz, yz = qx * qy, qx * qz, qy * qz
    wx, wy, wz = qw * qx, qw * qy, qw * qz

    R = np.array([
        [1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy)],
        [2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx)],
        [2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy)]
    ])
    return R


def quaternion_to_euler_angles(qx, qy, qz, qw):
    """
    Convert quaternion to Euler angles (roll, pitch, yaw)
    Returns angles in degrees
    """
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (qw * qx + qy * qz)
    cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2 * (qw * qy - qz * qx)
    if abs(sinp) >= 1:
        pitch = np.copysign(np.pi / 2, sinp)  # use 90 degrees if out of range
    else:
        pitch = np.arcsin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2 * (qw * qz + qx * qy)
    cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    # Convert to degrees
    return np.degrees(roll), np.degrees(pitch), np.degrees(yaw)


def pose_to_transform_matrix(position, orientation):
    """Convert pose (position + quaternion) to 4x4 transform matrix"""
    x, y, z = position['x'], position['y'], position['z']
    qx, qy, qz, qw = orientation['x'], orientation['y'], orientation['z'], orientation['w']

    R = quaternion_to_rotation_matrix(qx, qy, qz, qw)

    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [x, y, z]

    return T


def draw_grid_y(resolution=2., center=np.array([0, 0, 0])):
    """Draw grid on XY plane (Z=0 plane) for top-down view"""
    num_cells = 100

    gl.glLineWidth(2)
    set_gl_color(Color.kBlack)

    row_starts = []
    row_ends = []
    col_starts = []
    col_ends = []
    for idx in range(-num_cells, num_cells + 1):
        delta = idx * resolution
        # Grid on XY plane (Z=0)
        row_starts.append(np.array([delta, -num_cells * resolution, 0]) + center)
        row_ends.append(np.array([delta, num_cells * resolution, 0]) + center)

        col_starts.append(np.array([-num_cells * resolution, delta, 0]) + center)
        col_ends.append(np.array([num_cells * resolution, delta, 0]) + center)

    pangolin.DrawLines(row_starts, row_ends)
    pangolin.DrawLines(col_starts, col_ends)


def draw_pose(tf_world_key, length):
    """Draw pose as coordinate axes"""
    o = transform_points(tf_world_key, np.array([0, 0, 0]))
    x = transform_points(tf_world_key, np.array([length, 0, 0]))
    y = transform_points(tf_world_key, np.array([0, length, 0]))
    z = transform_points(tf_world_key, np.array([0, 0, length]))
    set_gl_color(Color.kRed)
    pangolin.DrawLine([o, x])
    set_gl_color(Color.kGreen)
    pangolin.DrawLine([o, y])
    set_gl_color(Color.kBlue)
    pangolin.DrawLine([o, z])


def draw_world_frame(length=10.0, line_width=3):
    """
    Draw world coordinate frame at origin
    X-axis: Red
    Y-axis: Green
    Z-axis: Blue
    """
    origin = np.array([0, 0, 0])
    x_end = np.array([length, 0, 0])
    y_end = np.array([0, length, 0])
    z_end = np.array([0, 0, length])

    gl.glLineWidth(line_width)

    # X-axis (Red)
    set_gl_color(Color.kRed)
    pangolin.DrawLine([origin, x_end])

    # Y-axis (Green)
    set_gl_color(Color.kGreen)
    pangolin.DrawLine([origin, y_end])

    # Z-axis (Blue)
    set_gl_color(Color.kBlue)
    pangolin.DrawLine([origin, z_end])


def draw_arrow(start, end, line_width=2, color=Color.kBlack):
    """Draw an arrow from start to end"""
    set_gl_color(color)
    gl.glLineWidth(line_width)
    pangolin.DrawLine([start, end])

    arrow_length = np.linalg.norm(np.array(end) - np.array(start))
    if arrow_length < 0.001:
        return

    arrow_head_length = 0.2 * arrow_length
    arrow_head_angle = np.pi / 6

    direction = np.array(end) - np.array(start)
    direction = direction / np.linalg.norm(direction)

    # Find perpendicular vector
    if abs(direction[2]) < 0.9:
        perpendicular = np.cross(direction, np.array([0, 0, 1]))
    else:
        perpendicular = np.cross(direction, np.array([1, 0, 0]))
    perpendicular = perpendicular / np.linalg.norm(perpendicular)

    arrow_head_point1 = np.array(end) - arrow_head_length * (np.cos(arrow_head_angle) * direction + np.sin(arrow_head_angle) * perpendicular)
    arrow_head_point2 = np.array(end) - arrow_head_length * (np.cos(arrow_head_angle) * direction - np.sin(arrow_head_angle) * perpendicular)

    pangolin.DrawLine([arrow_head_point1, end])
    pangolin.DrawLine([arrow_head_point2, end])


def draw_uncertainty_ellipse(position, covariance_3x3, scale=3.0, color=Color.kYellow):
    """
    Draw uncertainty ellipse at position
    covariance_3x3: 3x3 position covariance matrix
    scale: scale factor for visualization (e.g., 3 sigma)
    """
    try:
        # Eigenvalue decomposition
        eigenvalues, eigenvectors = np.linalg.eigh(covariance_3x3)

        # Sort by eigenvalue magnitude
        idx = eigenvalues.argsort()[::-1]
        eigenvalues = eigenvalues[idx]
        eigenvectors = eigenvectors[:, idx]

        # Scale eigenvalues for visualization
        radii = np.sqrt(eigenvalues) * scale

        # Draw ellipse using lines
        set_gl_color(color)
        gl.glLineWidth(1)

        # Draw principal axes
        for i in range(3):
            axis = eigenvectors[:, i] * radii[i]
            start = position - axis
            end = position + axis
            pangolin.DrawLine([start, end])

        # Draw ellipse outline
        num_points = 20
        for axis_idx in range(3):
            points = []
            for i in range(num_points):
                angle = 2 * np.pi * i / num_points
                if axis_idx == 0:  # YZ plane
                    offset = radii[1] * np.cos(angle) * eigenvectors[:, 1] + radii[2] * np.sin(angle) * eigenvectors[:, 2]
                elif axis_idx == 1:  # XZ plane
                    offset = radii[0] * np.cos(angle) * eigenvectors[:, 0] + radii[2] * np.sin(angle) * eigenvectors[:, 2]
                else:  # XY plane
                    offset = radii[0] * np.cos(angle) * eigenvectors[:, 0] + radii[1] * np.sin(angle) * eigenvectors[:, 1]

                point = position + offset
                points.append(point)

            # Draw circle
            for i in range(len(points)):
                pangolin.DrawLine([points[i], points[(i+1) % len(points)]])

    except Exception as e:
        # Fallback: draw simple sphere approximation
        set_gl_color(color)
        gl.glLineWidth(1)
        radius = scale * 0.1
        for i in range(3):
            axis = np.array([radius if j == i else 0 for j in range(3)])
            start = position - axis
            end = position + axis
            pangolin.DrawLine([start, end])


def TopViewY(center=np.array([0, 0, 0])):
    """Top view looking down along Z axis, with Z-axis pointing up"""
    scam = pangolin.OpenGlRenderState(
        pangolin.ProjectionMatrix(1920, 1080, 2000, 2000, 960, 540, 0.1, 500),
        pangolin.ModelViewLookAt(center[0], center[1], center[2] + 150,
                                 center[0], center[1], center[2],
                                 0, 1, 0))
    return scam


def TopViewFV(center=np.array([0, 0, 0])):
    """Perspective view"""
    scam = pangolin.OpenGlRenderState(
        pangolin.ProjectionMatrix(1920, 1080, 2000, 2000, 960, 540, 0.1, 500),
        pangolin.ModelViewLookAt(center[0] - 20, center[1] - 20, center[2] + 15,
                                 center[0] + 10, center[1] + 10, center[2],
                                 0, 0, 1))
    return scam


def load_odometry_data(data_dir):
    """Load all odometry data from directory"""
    odometry_dir = os.path.join(data_dir, 'asset_data', 'odometry')
    files = natsorted(glob.glob(os.path.join(odometry_dir, '*.json')))

    data_list = []
    for file_path in files:
        with open(file_path, 'r') as f:
            data = json.load(f)
            data_list.append(data)

    return data_list


def load_state_other_data(data_dir):
    """Load all state_other data from directory"""
    state_dir = os.path.join(data_dir, 'asset_data', 'state_other')
    files = natsorted(glob.glob(os.path.join(state_dir, '*.json')))

    data_list = []
    for file_path in files:
        with open(file_path, 'r') as f:
            data = json.load(f)
            data_list.append(data)

    return data_list


def create_text_image(frame_idx, frame_size, odometry_data, state_data, frame_distances,
                      width=320, height=540):
    """Create text information image with units and dark color scheme"""
    txt_image = np.ones((height, width, 3), dtype=np.uint8) * 240

    idx = 0

    # Frame info - Dark Blue
    txt = f"Frame: {frame_idx}/{frame_size-1}"
    idx += 1
    cv2.putText(txt_image, txt, (10, 40 * idx),
               4, 0.8, color2bgr(Color.kDarkBlue), 2)

    # Timestamp - Dark Gray
    if frame_idx < len(odometry_data):
        timestamp = odometry_data[frame_idx]['timestamp']
        txt = f"Time: {timestamp:.2f}"
        idx += 1
        cv2.putText(txt_image, txt, (10, 40 * idx),
                   4, 0.8, color2bgr(Color.kDarkGray), 2)

    # Frame distance (cumulative chord length) - Dark Green
    if frame_idx < len(frame_distances):
        dist = frame_distances[frame_idx]
        txt = f"Dist: {dist:.3f} m"
        idx += 1
        cv2.putText(txt_image, txt, (10, 40 * idx),
                   4, 0.8, color2bgr(Color.kDarkGreen), 2)

    # Position z value - Dark Red
    if frame_idx < len(odometry_data):
        pos = odometry_data[frame_idx]['pose']['position']
        txt = f"z: {pos['z']:.3f} m"
        idx += 1
        cv2.putText(txt_image, txt, (10, 40 * idx),
                   4, 0.8, color2bgr(Color.kDarkRed), 2)

    # Velocity magnitude - Dark Cyan
    if frame_idx < len(odometry_data):
        twist = odometry_data[frame_idx]['twist']
        vel = twist['linear']
        vel_vector = np.array([vel['x'], vel['y'], vel['z']])
        vel_mag = np.linalg.norm(vel_vector)
        txt = f"|v|: {vel_mag:.3f} m/s"
        idx += 1
        cv2.putText(txt_image, txt, (10, 40 * idx),
                   4, 0.8, color2bgr(Color.kDarkCyan), 2)

    # Angular velocity magnitude - Dark Orange
    if frame_idx < len(odometry_data):
        twist = odometry_data[frame_idx]['twist']
        ang_vel = twist['angular']
        ang_vel_vector = np.array([ang_vel['x'], ang_vel['y'], ang_vel['z']])
        ang_vel_mag = np.linalg.norm(ang_vel_vector)
        txt = f"|w|: {ang_vel_mag:.3f} rad/s"
        idx += 1
        cv2.putText(txt_image, txt, (10, 40 * idx),
                   4, 0.8, color2bgr(Color.kDarkOrange), 2)

    # Gravity magnitude - Dark Magenta
    if frame_idx < len(state_data):
        grav = state_data[frame_idx]['grav']
        grav_vector = np.array([grav['x'], grav['y'], grav['z']])
        grav_mag = np.linalg.norm(grav_vector)
        txt = f"|g|: {grav_mag:.3f} m/s^2"
        idx += 1
        cv2.putText(txt_image, txt, (10, 40 * idx),
                   4, 0.8, color2bgr(Color.kDarkMagenta), 2)

    # Accelerometer bias magnitude - Dark Yellow
    if frame_idx < len(state_data):
        ba = state_data[frame_idx]['ba']
        ba_vector = np.array([ba['x'], ba['y'], ba['z']])
        ba_mag = np.linalg.norm(ba_vector)
        txt = f"|ba|: {ba_mag:.3f} m/s^2"
        idx += 1
        cv2.putText(txt_image, txt, (10, 40 * idx),
                   4, 0.8, color2bgr(Color.kDarkYellow), 2)

    # Gyroscope bias magnitude - Dark Red
    if frame_idx < len(state_data):
        bg = state_data[frame_idx]['bg']
        bg_vector = np.array([bg['x'], bg['y'], bg['z']])
        bg_mag = np.linalg.norm(bg_vector)
        txt = f"|bg|: {bg_mag:.3f} rad/s"
        idx += 1
        cv2.putText(txt_image, txt, (10, 40 * idx),
                   4, 0.8, color2bgr(Color.kDarkRed), 2)

    return txt_image


def calculate_frame_distances(positions):
    """
    Calculate cumulative chord length (frame distance) from positions
    First frame is 0.0, others are cumulative distance
    """
    if len(positions) == 0:
        return np.array([])

    distances = np.zeros(len(positions))

    for i in range(1, len(positions)):
        # Calculate chord length between consecutive frames
        chord_length = np.linalg.norm(positions[i] - positions[i-1])
        distances[i] = distances[i-1] + chord_length

    return distances


def parse_launch_yaml(launch_path):
    """
    Parse launch.yaml to get Tbl (base to lidar transform)
    Returns: 4x4 transformation matrix
    """
    with open(launch_path, 'r') as f:
        content = f.read()

    # Find the static_transform_publisher command for base->velodyne_link
    # Pattern: "ros2 run tf2_ros static_transform_publisher x y z roll pitch yaw frame_id child_frame_id"
    pattern = r'static_transform_publisher\s+([\d.-]+)\s+([\d.-]+)\s+([\d.-]+)\s+([\d.-]+)\s+([\d.-]+)\s+([\d.-]+)\s+.*velodyne_link'

    match = re.search(pattern, content)
    if match:
        x, y, z, roll, pitch, yaw = [float(match.group(i)) for i in range(1, 7)]

        # Convert Euler angles to rotation matrix (ZYX order: yaw, pitch, roll)
        cr, sr = np.cos(roll), np.sin(roll)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cy, sy = np.cos(yaw), np.sin(yaw)

        R = np.array([
            [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
            [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
            [-sp, cp*sr, cp*cr]
        ])

        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = [x, y, z]

        return T

    # Default identity if not found
    return np.eye(4)


def parse_config_yaml(config_path):
    """
    Parse config.yaml to get Tli (lidar to imu transform)
    Returns: 4x4 transformation matrix
    """
    with open(config_path, 'r') as f:
        content = f.read()

    # Find extrinsic_T and extrinsic_R
    t_pattern = r'extrinsic_T:\s*\[\s*([\d.eE+-]+)\s*,\s*([\d.eE+-]+)\s*,\s*([\d.eE+-]+)\s*\]'
    r_pattern = r'extrinsic_R:\s*\[\s*([\d.eE+-]+)\s*,\s*([\d.eE+-]+)\s*,\s*([\d.eE+-]+)\s*,\s*([\d.eE+-]+)\s*,\s*([\d.eE+-]+)\s*,\s*([\d.eE+-]+)\s*,\s*([\d.eE+-]+)\s*,\s*([\d.eE+-]+)\s*,\s*([\d.eE+-]+)\s*\]'

    t_match = re.search(t_pattern, content)
    r_match = re.search(r_pattern, content)

    if t_match and r_match:
        # Translation
        t = [float(t_match.group(i)) for i in range(1, 4)]

        # Rotation (row-major order)
        r_values = [float(r_match.group(i)) for i in range(1, 10)]
        R = np.array(r_values).reshape(3, 3)

        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = t

        return T

    # Default identity if not found
    return np.eye(4)


def draw_coordinate_frame(T, length, line_width=3):
    """
    Draw coordinate frame axes
    T: 4x4 transformation matrix
    length: length of each axis
    X-axis: Red, Y-axis: Green, Z-axis: Blue
    """
    origin = T[:3, 3]
    R = T[:3, :3]

    x_end = origin + R[:, 0] * length
    y_end = origin + R[:, 1] * length
    z_end = origin + R[:, 2] * length

    gl.glLineWidth(line_width)

    # X-axis (Red)
    set_gl_color(Color.kRed)
    pangolin.DrawLine([origin, x_end])

    # Y-axis (Green)
    set_gl_color(Color.kGreen)
    pangolin.DrawLine([origin, y_end])

    # Z-axis (Blue)
    set_gl_color(Color.kBlue)
    pangolin.DrawLine([origin, z_end])
