import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'urdf_tuto'

    # Cartographer 설정 파일 경로
    cartographer_config_dir = os.path.join(get_package_share_directory(package_name), 'config')
    cartographer_config_basename = 'cartographer.lua'

    # Cartographer 노드
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '-configuration_directory', cartographer_config_dir,
            '-configuration_basename', cartographer_config_basename
        ],
        # URDF에서 설정한 라이다 토픽 이름이 'scan'이므로 그대로 매핑 [cite: 19]
        remappings=[('scan', '/scan')]
    )

    # 지도를 2D Occupancy Grid(우리가 아는 그 흑백 지도)로 변환해 주는 노드
    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': True},
                    {'resolution': 0.05}]
    )

    return LaunchDescription([
        cartographer_node,
        occupancy_grid_node
    ])