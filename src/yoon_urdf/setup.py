import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'yoon_urdf'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yoon',
    maintainer_email='yoon@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pedestrian_mock_node = yoon_urdf.pedestrian_mock_node:main',
            'pedestrian_detector = yoon_urdf.pedestrian_detector:main',
            'feature_mapper = yoon_urdf.feature_mapper:main',
            'risk_map_generator = yoon_urdf.risk_map_generator:main',
            'mujoco_cam_publisher = yoon_urdf.mujoco_cam_publisher:main',
        ],
    },
)
