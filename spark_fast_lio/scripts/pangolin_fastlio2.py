#!/usr/bin/env python3
"""
FastLIO2数据可视化工具
使用Pangolin可视化pose、旋转、重力等关键数据
"""

import cv2
import numpy as np
import sys
import os
import time

# Add utils to path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# Pangolin and OpenGL
import pangolin
import OpenGL.GL as gl

# Import utilities
from utils.visualization_utils import (
    Color, color2bgr, set_gl_color, transform_points,
    quaternion_to_rotation_matrix, quaternion_to_euler_angles, pose_to_transform_matrix,
    draw_grid_y, draw_pose, draw_arrow,
    TopViewY, TopViewFV,
    load_odometry_data, load_state_other_data,
    create_text_image, calculate_frame_distances,
    parse_launch_yaml, parse_config_yaml, draw_coordinate_frame
)


def main(data_dir, launch_yaml, config_yaml):
    # Data directory (should point to the bag directory, not asset_data)
    # Parse transform parameters
    print("Parsing configuration files...")
    Tbl = parse_launch_yaml(launch_yaml)  # Base to LiDAR transform
    Tli = parse_config_yaml(config_yaml)
    Tli = np.linalg.inv(Tli)
    Tbi = Tbl @ Tli  # Base to IMU transform
    print(f"Tbl (Base->LiDAR):\n{Tbl}")
    print(f"Tli (LiDAR->IMU):\n{Tli}")
    print(f"Tbi (Base->IMU):\n{Tbi}")

    print("Loading odometry data...")
    odometry_data = load_odometry_data(data_dir)
    print(f"Loaded {len(odometry_data)} odometry frames")

    print("Loading state_other data...")
    state_data = load_state_other_data(data_dir)
    print(f"Loaded {len(state_data)} state frames")

    if len(odometry_data) == 0:
        print("No data found, exiting")
        return

    # Extract Twb0 from first state_other frame
    Twb0 = np.eye(4)
    if len(state_data) > 0 and 'Twb0' in state_data[0]:
        twb0_data = state_data[0]['Twb0']
        Twb0 = np.array([
            [twb0_data['r00'], twb0_data['r01'], twb0_data['r02'], 0],
            [twb0_data['r10'], twb0_data['r11'], twb0_data['r12'], 0],
            [twb0_data['r20'], twb0_data['r21'], twb0_data['r22'], 0],
            [0, 0, 0, 1]
        ])
        print(f"Twb0 (gravity aligned):\n{Twb0}")
    else:
        print("Warning: Twb0 not found in state_data, using identity")

    # Calculate Twl0 and Twi0 from Twb0
    # Twl0 = Twb0 * Tbl (world to LiDAR at initial frame)
    # Twi0 = Twb0 * Tbi (world to IMU at initial frame)
    Twl0 = Twb0 @ Tbl
    Twi0 = Twb0 @ Tbi
    print(f"Twl0 (World->LiDAR0):\n{Twl0}")
    print(f"Twi0 (World->IMU0):\n{Twi0}")

    # Extract trajectory
    positions = []
    orientations = []
    covariances = []

    for data in odometry_data:
        pos = data['pose']['position']
        ori = data['pose']['orientation']
        cov = data['pose']['covariance']

        positions.append([pos['x'], pos['y'], pos['z']])
        orientations.append([ori['x'], ori['y'], ori['z'], ori['w']])

        # Convert 36-element covariance to 6x6 matrix
        cov_matrix = np.array(cov).reshape(6, 6)
        covariances.append(cov_matrix)

    positions = np.array(positions)
    orientations = np.array(orientations)

    # Calculate frame distances (cumulative chord length)
    print("Calculating frame distances...")
    frame_distances = calculate_frame_distances(positions)
    print(f"Total distance: {frame_distances[-1]:.3f} m")

    # Calculate center for view
    center = np.mean(positions, axis=0) if len(positions) > 0 else np.zeros(3)

    frame_size = len(odometry_data)
    current_frame_idx = 0

    # Screen parameters
    screen_w = 1920
    screen_h = 1080

    pangolin.CreateWindowAndBind('FastLIO2 Visualization', screen_w, screen_h)
    gl.glEnable(gl.GL_DEPTH_TEST)
    gl.glEnable(gl.GL_BLEND)
    gl.glBlendFunc(gl.GL_SRC_ALPHA, gl.GL_ONE_MINUS_SRC_ALPHA)

    # Camera setup
    scam = pangolin.OpenGlRenderState(
        pangolin.ProjectionMatrix(screen_w, screen_h, 2000, 2000, 960, 540, 0.1, 500),
        pangolin.ModelViewLookAt(center[0], center[1] + 150, center[2],
                                 center[0], center[1], center[2],
                                 0, 0, 1))
    handler = pangolin.Handler3D(scam)

    dcam = pangolin.CreateDisplay()
    dcam.SetBounds(0.0, 1.0, 0.0, 1.0, -screen_w / screen_h)
    dcam.SetHandler(handler)

    # Text image panel (left bottom)
    txt_b, txt_t, txt_l, txt_r = 0.0, 0.5, 0.0, 1. / 6.
    txt_imageh, txt_imagew = 540, 320
    dtxtimg = pangolin.Display('txt')
    dtxtimg.SetBounds(txt_b, txt_t, txt_l, txt_r, float(txt_imagew) / float(txt_imageh))
    dtxtimg.SetLock(pangolin.Lock.LockLeft, pangolin.Lock.LockTop)
    txt_text = pangolin.GlTexture(txt_imagew, txt_imageh, gl.GL_RGB, False, 0, gl.GL_RGB, gl.GL_UNSIGNED_BYTE)

    # Control panel (left top)
    panel = pangolin.CreatePanel('ui')
    panel.SetBounds(0.5, 1., 0.0, 1. / 6.)

    show_top_view = pangolin.VarBool('ui.TopView', value=True, toggle=False)
    show_grid = pangolin.VarBool('ui.grid', value=True, toggle=True)
    show_Twb0 = pangolin.VarBool('ui.Twb0', value=True, toggle=True)
    show_Twl0 = pangolin.VarBool('ui.Twl0', value=False, toggle=True)
    show_Twi0 = pangolin.VarBool('ui.Twi0', value=False, toggle=True)
    show_trajectory = pangolin.VarBool('ui.trajectory', value=True, toggle=True)
    show_current_pose = pangolin.VarBool('ui.curr_pose (Twi)', value=True, toggle=True)
    show_gravity = pangolin.VarBool('ui.gravity', value=True, toggle=True)
    auto_play = pangolin.VarBool('ui.Auto Play', value=False, toggle=False)
    play_step = pangolin.VarBool('ui.>>', value=False, toggle=False)
    play_back = pangolin.VarBool('ui.<<', value=False, toggle=False)
    curr_frame_idx = pangolin.VarInt('ui.frame_idx', value=0, min=0, max=frame_size - 1)
    pose_length = pangolin.VarInt('ui.pose_len', value=3, min=1, max=10)

    print("Starting visualization loop...")

    # Auto play timing
    last_frame_time = 0
    frame_interval = 0.05  # 20Hz

    while not pangolin.ShouldQuit():
        current_time = time.time()

        gl.glClear(gl.GL_COLOR_BUFFER_BIT | gl.GL_DEPTH_BUFFER_BIT)
        gray_color = 125. / 255.
        gl.glClearColor(gray_color, gray_color, gray_color, 1.0)
        dcam.Activate(scam)

        frame_idx = curr_frame_idx.Get()

        # Auto play control
        if auto_play.Get():
            if current_time - last_frame_time >= frame_interval:
                frame_idx = (frame_idx + 1) % frame_size
                curr_frame_idx.SetVal(frame_idx)
                last_frame_time = current_time
        else:
            # Manual control
            if play_step.Get():
                play_step.SetVal(False)
                frame_idx = (frame_idx + 1) % frame_size

            if play_back.Get():
                play_back.SetVal(False)
                frame_idx = (frame_idx - 1) % frame_size

            curr_frame_idx.SetVal(frame_idx)

        # View control
        if show_top_view.Get():
            show_top_view.SetVal(False)
            scam = TopViewY(center)
            dcam.SetHandler(pangolin.Handler3D(scam))

        # Draw grid
        if show_grid.Get():
            draw_grid_y(10., center)

        # Draw coordinate frames
        # Twb0: World to Base0 frame (gravity aligned), 4m, default ON
        if show_Twb0.Get():
            draw_coordinate_frame(Twb0, length=4.0, line_width=3)

        # Twl0: World to LiDAR0 frame, 3m, default OFF
        if show_Twl0.Get():
            draw_coordinate_frame(Twl0, length=3.0, line_width=3)

        # Twi0: World to IMU0 frame, 2m, default OFF
        if show_Twi0.Get():
            draw_coordinate_frame(Twi0, length=2.0, line_width=3)

        # Draw trajectory
        if show_trajectory.Get() and len(positions) > 1:
            set_gl_color(Color.kCyan)
            gl.glLineWidth(2)

            # Draw trajectory line
            traj_points = positions[:frame_idx+1]
            if len(traj_points) > 1:
                for i in range(len(traj_points) - 1):
                    pangolin.DrawLine([traj_points[i], traj_points[i+1]])

        # Draw start point (first frame position) as a large black point
        if len(positions) > 0:
            set_gl_color(Color.kBlack)
            gl.glPointSize(15.0)  # Large point size
            pangolin.DrawPoints([positions[0]])

        # Draw current pose
        if show_current_pose.Get() and frame_idx < len(positions):
            current_pos = positions[frame_idx]
            current_ori = orientations[frame_idx]

            # Create transform matrix
            T = np.eye(4)
            R = quaternion_to_rotation_matrix(current_ori[0], current_ori[1], current_ori[2], current_ori[3])
            T[:3, :3] = R
            T[:3, 3] = current_pos

            # Draw pose axes
            draw_pose(T, pose_length.Get())

        # Draw gravity vector
        if show_gravity.Get() and frame_idx < len(state_data):
            state = state_data[frame_idx]
            grav = state['grav']

            # Get current position
            if frame_idx < len(positions):
                current_pos = positions[frame_idx]

                # Gravity vector (normalized and scaled to 3m for visualization)
                grav_vector = np.array([grav['x'], grav['y'], grav['z']])
                grav_norm = grav_vector / np.linalg.norm(grav_vector)
                grav_end = current_pos + grav_norm * 3.0  # Fixed 3m length

                # Draw gravity arrow
                draw_arrow(current_pos, grav_end, line_width=3, color=Color.kMagenta)

        # Create and draw text information
        txt_image = create_text_image(
            frame_idx, frame_size, odometry_data, state_data, frame_distances
        )

        # Convert to RGB and upload
        txt_image_rgb = cv2.cvtColor(txt_image, cv2.COLOR_BGR2RGB)
        txt_text.Upload(txt_image_rgb, gl.GL_RGB, gl.GL_UNSIGNED_BYTE)
        dtxtimg.Activate()
        gl.glColor3f(1.0, 1.0, 1.0)
        txt_text.RenderToViewportFlipY()

        pangolin.FinishFrame()


if __name__ == '__main__':
    # Default parameters
    data_dir = "/home/jerett/OpenProject/LidarSlam/spark-fast-lio/spark_fast_lio/scripts/data/lio_20260429_213020"
    launch_yaml = "/home/jerett/OpenProject/LidarSlam/spark-fast-lio/spark_fast_lio/launch/mapping_mit_campus.launch.yaml"
    config_yaml = "/home/jerett/OpenProject/LidarSlam/spark-fast-lio/spark_fast_lio/config/velodyne_mit.yaml"

    main(data_dir, launch_yaml, config_yaml)
