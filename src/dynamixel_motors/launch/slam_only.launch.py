from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="lidar_tf",
            arguments=["--x","0.06","--y","0","--z","0.15",
                       "--roll","0","--pitch","0","--yaw","0",
                       "--frame-id","base_link","--child-frame-id","laser_frame"],
        ),

        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="odom_tf",
            arguments=["--x","0","--y","0","--z","0",
                       "--roll","0","--pitch","0","--yaw","0",
                       "--frame-id","odom","--child-frame-id","base_link"],
        ),

        Node(
            package="ydlidar_ros2_driver",
            executable="ydlidar_ros2_driver_node",
            name="ydlidar_node",
            output="screen",
            parameters=[{
                "port": "/dev/ttyUSB0",
                "baudrate": 128000,
                "lidar_type": 1,
                "device_type": 6,
                "sample_rate": 5,
                "fixed_resolution": False,
                "reversion": True,
                "inverted": False,
                "auto_reconnect": True,
                "isSingleChannel": False,
                "intensity": False,
                "support_motor_dtr": True,
                "angle_max": 180.0,
                "angle_min": -180.0,
                "range_max": 10.0,
                "range_min": 0.12,
                "frequency": 10.0,
                "invalid_range_is_inf": False,
                "frame_id": "laser_frame",
            }],
        ),

        Node(
            package="dynamixel_motors",
            executable="dynamixel_node",
            name="dynamixel_node",
            output="screen",
            parameters=[{
                "port": "/dev/ttyUSB1",
                "baudrate": 115200,
                "servo_ids": [2],
            }],
        ),

        Node(
            package="dynamixel_motors",
            executable="odom_relay",
            name="odom_relay",
            output="screen",
        ),

        Node(
            package="dynamixel_motors",
            executable="cmd_vel_bridge",
            name="cmd_vel_bridge",
            output="screen",
        ),

        Node(
            package="slam_toolbox",
            executable="sync_slam_toolbox_node",
            name="slam_toolbox",
            output="screen",
            parameters=[{
                "use_sim_time": False,
                "base_frame": "base_link",
                "odom_frame": "odom",
                "map_frame": "map",
                "scan_topic": "/scan",
                "minimum_travel_distance": 0.001,
                "minimum_travel_heading": 0.001,
                "map_update_interval": 0.2,
                "transform_publish_period": 0.02,
                "resolution": 0.05,
                "max_laser_range": 10.0,
            }],
        ),
    ])
