#!/usr/bin/env python3
"""
robot.launch.py — FIXED
Changes vs previous version:
  1. Added robot_state_publisher (loads robot.urdf → enables RViz RobotModel)
  2. Removed static_transform_publisher for laser_frame
     (laser_joint is now defined inside the URDF as a fixed joint)
  3. Nav2 timer kept at 15 s — SLAM needs time to build first map
"""
import os
from launch import LaunchDescription
from launch.actions import TimerAction, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dyn  = get_package_share_directory('dynamixel_motors')
    pkg_nav2 = get_package_share_directory('nav2_bringup')

    # ── Load URDF ─────────────────────────────────────────────────────
    urdf_file = os.path.join(pkg_dyn, 'urdf', 'robot.urdf')
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # ── Nav2 launch ───────────────────────────────────────────────────
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time':       'False',
            'autostart':          'True',
            'use_docking_server': 'False',
            'params_file':        os.path.join(pkg_dyn, 'config', 'nav2_params.yaml'),
        }.items(),
    )

    return LaunchDescription([

        # ── Robot description (URDF → /robot_description) ─────────────
        # This enables the RobotModel display in RViz and publishes
        # the base_link → laser_frame fixed TF (no separate static_tf needed).
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description,
                         'use_sim_time': False}],
            output='screen',
        ),

        # NOTE: static_transform_publisher for laser_frame is REMOVED.
        # The laser_joint in robot.urdf now handles that transform.
        # robot_state_publisher publishes all fixed joints automatically.

        # ── YDLIDAR ───────────────────────────────────────────────────
        Node(
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            name='ydlidar_ros2_driver_node',
            parameters=[os.path.join(pkg_dyn, 'config', 'ydlidar.yaml')],
        ),

        # ── Dynamixel servo driver ────────────────────────────────────
        Node(
            package='dynamixel_motors',
            executable='dynamixel_node',
            name='dynamixel_node',
            parameters=[os.path.join(pkg_dyn, 'config', 'motors.yaml')],
        ),

        # ── Wheel odometry + TF broadcaster + /joint_states ──────────
        Node(
            package='dynamixel_motors',
            executable='odom_pub',
            name='odom_pub',
        ),

        # ── cmd_vel → servo RPM bridge ────────────────────────────────
        Node(
            package='dynamixel_motors',
            executable='diff_drive_controller',
            name='diff_drive_controller',
        ),

        # ── SLAM Toolbox (owns map → odom TF) ─────────────────────────
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='sync_slam_toolbox_node',
            parameters=[
                os.path.join(pkg_dyn, 'config', 'slam_toolbox.yaml'),
                {'use_sim_time': False},
            ],
        ),

        # Auto-configure and activate slam_toolbox
        ExecuteProcess(
            cmd=['bash', '-c',
                'until ros2 lifecycle get /sync_slam_toolbox_node 2>/dev/null '
                '    | grep -q unconfigured; '
                'do echo "[slam_wait] waiting for slam_toolbox..."; sleep 1; done; '
                'echo "[slam_wait] configuring..."; '
                'ros2 lifecycle set /sync_slam_toolbox_node configure; '
                'sleep 2; '
                'echo "[slam_wait] activating..."; '
                'ros2 lifecycle set /sync_slam_toolbox_node activate; '
                'echo "[slam_wait] slam_toolbox active!"'
            ],
            output='screen',
        ),

        # Nav2 — delayed 15 s to let SLAM establish the map → odom TF
        TimerAction(period=15.0, actions=[nav2_launch]),
    ])
