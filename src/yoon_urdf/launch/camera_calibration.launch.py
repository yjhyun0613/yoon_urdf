import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. Isolated MuJoCo Calibration Publisher
        Node(
            package='yoon_urdf',
            executable='mujoco_calibration_publisher',
            name='mujoco_calibration_publisher',
            output='screen'
        ),

        # 2. Auto Calibration Test Node
        Node(
            package='yoon_urdf',
            executable='test_calibration_auto',
            name='test_calibration_auto',
            output='screen'
        )
    ])
