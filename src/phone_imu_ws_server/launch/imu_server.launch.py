import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='phone_imu_ws_server',
            executable='imu_server',
            name='phone_imu_server',
            output='screen',
            parameters=[
                {'frame_id': 'phone_imu_link'}
            ]
        ),
    ])
