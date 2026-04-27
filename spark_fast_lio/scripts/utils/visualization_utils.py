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


def pose_to_transform_matrix(position, orientation):
    """Convert pose (position + quaternion) to 4x4 transform matrix"""
    x, y, z = position['x'], position['y'], position['z']
    qx, qy, qz, qw = orientation['x'], orientation['y'], orientation['z'], orientation['w']

    R = quaternion_to_rotation_matrix(qx, qy, qz, qw)

    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [x, y, z]

    return T


def draw_grid_y(resolution=1., center=np.array([0, 0, 0])):
    """Draw grid on Y plane (XZ plane)"""
    num_cells = 250

    gl.glLineWidth(2)
    set_gl_color(Color.kBlack)

    row_starts = []
    row_ends = []
    col_starts = []
    col_ends = []
    for idx in range(-num_cells, num_cells + 1):
        delta = idx * resolution
        row_starts.append(np.array([delta, 0, -num_cells * resolution]) + center)
        row_ends.append(np.array([delta, 0, num_cells * resolution]) + center)

        col_starts.append(np.array([-num_cells * resolution, 0, delta]) + center)
        col_ends.append(np.array([num_cells * resolution, 0, delta]) + center)

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
    """Top view looking down along Y axis"""
    scam = pangolin.OpenGlRenderState(
        pangolin.ProjectionMatrix(1920, 1080, 2000, 2000, 960, 540, 0.1, 200),
        pangolin.ModelViewLookAt(center[0], center[1] + 70, center[2],
                                 center[0], center[1], center[2],
                                 0, 0, 1))
    return scam


def TopViewFV(center=np.array([0, 0, 0])):
    """Perspective view"""
    scam = pangolin.OpenGlRenderState(
        pangolin.ProjectionMatrix(1920, 1080, 2000, 2000, 960, 540, 0.1, 200),
        pangolin.ModelViewLookAt(center[0] - 10, center[1] - 10, center[2] + 5,
                                 center[0] + 5, center[1] + 5, center[2],
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


def create_text_image(frame_idx, frame_size, odometry_data, positions, orientations,
                      state_data, covariances, width=320, height=540):
    """Create text information image"""
    txt_image = np.ones((height, width, 3), dtype=np.uint8) * 240

    idx = 0

    # Frame info
    txt = f"Frame: {frame_idx}/{frame_size-1}"
    idx += 1
    cv2.putText(txt_image, txt, (10, 40 * idx),
               4, 0.8, color2bgr(Color.kBlue), 2)

    # Timestamp
    if frame_idx < len(odometry_data):
        timestamp = odometry_data[frame_idx]['timestamp']
        txt = f"Time: {timestamp:.2f}"
        idx += 1
        cv2.putText(txt_image, txt, (10, 40 * idx),
                   4, 0.8, color2bgr(Color.kBlack), 2)

    # Position
    if frame_idx < len(positions):
        pos = positions[frame_idx]
        txt = f"Pos X: {pos[0]:.4f}"
        idx += 1
        cv2.putText(txt_image, txt, (10, 40 * idx),
                   4, 0.8, color2bgr(Color.kRed), 2)

        txt = f"Pos Y: {pos[1]:.4f}"
        idx += 1
        cv2.putText(txt_image, txt, (10, 40 * idx),
                   4, 0.8, color2bgr(Color.kGreen), 2)

        txt = f"Pos Z: {pos[2]:.4f}"
        idx += 1
        cv2.putText(txt_image, txt, (10, 40 * idx),
                   4, 0.8, color2bgr(Color.kBlue), 2)

    # Orientation
    if frame_idx < len(orientations):
        ori = orientations[frame_idx]
        txt = f"Quat w: {ori[3]:.6f}"
        idx += 1
        cv2.putText(txt_image, txt, (10, 40 * idx),
                   4, 0.8, color2bgr(Color.kBlack), 2)

        txt = f"Quat xyz: {ori[0]:.6f},{ori[1]:.6f},{ori[2]:.6f}"
        idx += 1
        cv2.putText(txt_image, txt, (10, 40 * idx),
                   4, 0.6, color2bgr(Color.kGray), 1)

    # Gravity
    if frame_idx < len(state_data):
        grav = state_data[frame_idx]['grav']
        txt = f"Grav X: {grav['x']:.4f}"
        idx += 1
        cv2.putText(txt_image, txt, (10, 40 * idx),
                   4, 0.8, color2bgr(Color.kMagenta), 2)

        txt = f"Grav Y: {grav['y']:.4f}"
        idx += 1
        cv2.putText(txt_image, txt, (10, 40 * idx),
                   4, 0.8, color2bgr(Color.kMagenta), 2)

        txt = f"Grav Z: {grav['z']:.4f}"
        idx += 1
        cv2.putText(txt_image, txt, (10, 40 * idx),
                   4, 0.8, color2bgr(Color.kMagenta), 2)

    # Uncertainty
    if frame_idx < len(covariances):
        pos_cov = covariances[frame_idx][:3, :3]
        uncertainty = np.trace(pos_cov)
        txt = f"Uncertainty: {uncertainty:.6e}"
        idx += 1
        cv2.putText(txt_image, txt, (10, 40 * idx),
                   4, 0.7, color2bgr(Color.kYellow), 2)

    return txt_image
