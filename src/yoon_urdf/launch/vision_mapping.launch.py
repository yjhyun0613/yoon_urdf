import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. MuJoCo Camera Publisher (Simulator)
        Node(
            package='yoon_urdf',
            executable='mujoco_cam_publisher',
            name='mujoco_cam_publisher',
            output='screen'
        ),

        # 2. Feature-Point Tracker and Triangulator Node
        Node(
            package='yoon_urdf',
            executable='feature_mapper',
            name='feature_mapper',
            output='screen'
        ),
        
        # 3. Risk Map Generator Node
        Node(
            package='yoon_urdf',
            executable='risk_map_generator',
            name='risk_map_generator',
            output='screen'
        )
    ])
