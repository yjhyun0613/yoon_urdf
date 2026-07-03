import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    package_name = "urdf_tuto"
    target_frame = LaunchConfiguration("target_frame")

    # Robot State Publisher for 3D Robot Model
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory(package_name), "launch", "robot_3d.launch.py")]
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py")]
        ),
        launch_arguments={'world': os.path.join(get_package_share_directory("urdf_tuto"), 'world', 'office_cpr.world')}.items(),
    )

    # Spawn Entity node
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "with_robot"],
        output="screen",
    )

    # RViz2 with default config (can add topics manually)
    rviz2 = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(get_package_share_directory('urdf_tuto'), 'config', 'robot2.rviz')],
        parameters=[{'use_sim_time': True}]
    )
    
    # Lidar-Camera Fusion Node
    fusion_node = Node(
        package=package_name,
        executable='lidar_camera_fusion',
        name='lidar_camera_fusion',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'target_frame': target_frame,
            'keep_only_visible': True
        }]
    )
    
    # PointCloud Accumulator and Saver Node
    accumulator_node = Node(
        package=package_name,
        executable='pointcloud_accumulator',
        name='pointcloud_accumulator',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'save_dir': '/home/yoon/yoon_urdf/saved_maps',
            'save_interval_sec': 15.0,
            'file_format': 'ply',
            'voxel_size': 0.02
        }]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "target_frame",
                default_value="odom",
                description="Coordinate frame to build the pointcloud map ('odom' or 'map')"
            ),
            rsp,
            gazebo,
            spawn_entity,
            rviz2,
            fusion_node,
            accumulator_node
        ]
    )
