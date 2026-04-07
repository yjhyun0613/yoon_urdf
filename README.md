ROS2 연습용 파일입니다.

- 실행 방법

cd yoon_urdf

colcon build

source install/setup.bash

ros2 launch urdf_tuto launch_sim.launch.py

  기본 런치 파일
  1. launch_sim.launch.py : teleopkey, 카메라, 라이다 적용
  
  욜로 적용 런치 파일
  1. fire_detect.launch.py : 불 감지 욜로 적용한 모델
  2. drone_detect.launch.py : 드론 감지 욜로 적용한 모델


- 변경
로봇 모델은 robot_3.launch.py에서 변경 가능
가제보 월드는 launch_sim.launch.py에서 gazebo 부분에서 변경 가능
