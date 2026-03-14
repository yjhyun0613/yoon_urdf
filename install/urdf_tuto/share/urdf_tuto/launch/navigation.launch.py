import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    package_name = 'urdf_tuto'
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # 패키지 설치 경로에서 맵 파일 찾기
    map_path = os.path.join(get_package_share_directory(package_name), 'maps', 'my_city_map.yaml')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'map': map_path,
                'use_sim_time': 'true',
            }.items(),
        ),
    ])