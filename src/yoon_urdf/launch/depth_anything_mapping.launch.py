import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. MuJoCo Camera Publisher (Monocular Simulator)
        Node(
            package='yoon_urdf',
            executable='mujoco_cam_publisher',
            name='mujoco_cam_publisher',
            output='screen'
        ),

        # 2. Depth Anything V2 Monocular AI Mapper Node
        Node(
            package='yoon_urdf',
            executable='depth_anything_mapper',
            name='depth_anything_mapper',
            output='screen',
            parameters=[{
                'downsample_step': 4,
                'min_depth': 0.5,
                'max_depth': 8.0,
                'model_id': 'depth-anything/Depth-Anything-V2-Small-hf'
            }]
        ),
        
        # 3. Risk Map Generator Node (Reused)
        Node(
            package='yoon_urdf',
            executable='risk_map_generator',
            name='risk_map_generator',
            output='screen'
        ),

        # 4. Point Cloud Saver and Accumulator Node (High Precision Voxel Grid)
        Node(
            package='yoon_urdf',
            executable='pointcloud_saver',
            name='pointcloud_saver',
            output='screen',
            parameters=[{
                'voxel_size': 0.02,
                'file_format': 'ply'
            }]
        )
    ])
