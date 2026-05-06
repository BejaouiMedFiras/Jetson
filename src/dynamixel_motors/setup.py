from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'dynamixel_motors'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # package index
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        # package.xml
        ('share/' + package_name, ['package.xml']),

        # launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),

        # config files
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),

        # URDF model — NEW (enables RobotModel in RViz)
        (os.path.join('share', package_name, 'urdf'),
            glob('urdf/*.urdf')),

        # rviz
        (os.path.join('share', package_name, 'rviz'),
            glob('rviz/*.rviz')),
    ],

    install_requires=['setuptools'],
    zip_safe=True,

    maintainer='root',
    maintainer_email='root@todo.todo',
    description='Dynamixel motors ROS2 node',
    license='MIT',

    entry_points={
        'console_scripts': [
            'dynamixel_node  = dynamixel_motors.dynamixel_node:main',
            'dynamixel_driver = dynamixel_motors.dynamixel_driver:main',
            'cmd_vel_bridge  = dynamixel_motors.cmd_vel_bridge:main',
            'odom_pub        = dynamixel_motors.odom_node:main',
            'scan_relay      = dynamixel_motors.scan_relay:main',
            'tf_publisher    = dynamixel_motors.tf_publisher:main',
            'diff_drive_controller = dynamixel_motors.diff_drive_controller:main',
        ],
    },
)
