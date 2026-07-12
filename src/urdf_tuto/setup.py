import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'urdf_tuto'

# Recursively collect all files from the models directory for Gazebo model:// URIs
def get_model_data_files(models_dir='models'):
    data_files = []
    for root, dirs, files in os.walk(models_dir):
        if files:
            install_dir = os.path.join('share', package_name, root)
            file_paths = [os.path.join(root, f) for f in files]
            data_files.append((install_dir, file_paths))
    return data_files

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'world'), glob('world/*.world')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'yolo'), glob('yolo/*.pt')),
        (os.path.join('share', package_name, 'fire_detect'), glob('fire_detect/*.py')),
        (os.path.join('share', package_name, 'drone_detector'), glob('drone_detector/*.py')),        
        (os.path.join('share', package_name, 'pointcloud_process'), glob('pointcloud_process/*.py')),        
        (os.path.join('share', package_name, 'maps'), glob('maps/*'))
    ] + get_model_data_files(),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yoon',
    maintainer_email='yoon@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_detector = yolo.yolo_detector:main',
            'fire_detector = fire_detect.fire_detector_01:main',
            'drone_detector = drone_detector.drone_detector_01:main',
            'lidar_camera_fusion = pointcloud_process.lidar_camera_fusion:main',
            'pointcloud_accumulator = pointcloud_process.pointcloud_accumulator:main',
        ],
    },
)
