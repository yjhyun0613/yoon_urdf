import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    package_name = "urdf_tuto"
    show_yolo_view = LaunchConfiguration("show_yolo_view")

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory(package_name), "launch", "robot_3.launch.py")]
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

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "with_robot"],
        output="screen",
    )

    rviz2 = Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(get_package_share_directory('urdf_tuto'), 'config', 'robot2.rviz')]
        )
    
    fire_detector_node = Node(
        package=package_name,
        executable='fire_detector', # setup.py에 등록한 이름
        output='screen',
        parameters=[{'use_sim_time': True}] # 가제보 시간과 동기화
    )
    
    # yolo_node = Node(
    # package=package_name,
    # executable='yolo_detector',
    # output='screen'
    #     )
    
    # yolo_image_view = Node(
    #     package="image_view",
    #     executable="image_view",
    #     name="yolo_image_view",
    #     remappings=[("image", "/yolo/annotated")],
    #     output="screen",
    #     )

    return LaunchDescription(
        [DeclareLaunchArgument(
            "show_yolo_view",
            default_value="true"
        ),
            rsp,
            gazebo,
            spawn_entity,
            rviz2,
            # yolo_node,
            # yolo_image_view
            fire_detector_node
        ]
    )