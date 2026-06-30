import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch argument for detection mode
    detection_mode_arg = DeclareLaunchArgument(
        'detection_mode',
        default_value='color',
        description="Detection mode: 'color' or 'yolo'"
    )
    
    detection_mode = LaunchConfiguration('detection_mode')

    return LaunchDescription([
        detection_mode_arg,

        # 1. MuJoCo RGB-D Camera Publisher (Simulator)
        Node(
            package='yoon_urdf',
            executable='mujoco_rgbd_publisher',
            name='mujoco_rgbd_publisher',
            output='screen'
        ),

        # 2. RGB-D Direct 3D Projection Mapper Node
        Node(
            package='yoon_urdf',
            executable='rgbd_mapper',
            name='rgbd_mapper',
            output='screen',
            parameters=[{'detection_mode': detection_mode}]
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
            output='screen'
        )
    ])
