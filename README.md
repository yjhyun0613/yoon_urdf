# ROS2 URDF 연습 프로젝트 (yoon_urdf)

이 저장소는 ROS2를 기반으로 URDF 로봇 모델링, Gazebo 시뮬레이션 환경 구축, 그리고 YOLO를 이용한 객체 탐지 기능을 실습하기 위해 만들어졌습니다.

## 🚀 실행 방법

아래 명령어를 순서대로 입력하여 빌드 및 시뮬레이션을 실행합니다.


### 1. 워크스페이스 이동
cd yoon_urdf

### 2. 패키지 빌드
colcon build

### 3. 환경 설정 적용
source install/setup.bash

### 4. 기본 시뮬레이션 실행
ros2 launch urdf_tuto launch_sim.launch.py

| 런치 파일 명 | 주요 기능 | 포함 사항 |
| :--- | :--- | :--- |
| launch_sim.launch.py | 기본 시뮬레이션 실행 | TeleopKey, 카메라, 라이다(Lidar) |
| fire_detect.launch.py | 불 감지 시뮬레이션 | 불(Fire) 감지 YOLO 모델 적용 |
| drone_detect.launch.py | 드론 감지 시뮬레이션 | 드론(Drone) 감지 YOLO 모델 적용 |
| launch_sim_3d.launch.py | 3D 컬러 맵핑 시뮬레이션 | Lidar-Camera Fusion, 3D 점군 지도 누적 및 파일 저장 |


---

## 📂 상세 기능 가이드 (하위 문서 링크)

이 프로젝트의 상세 구성 및 각 노드별 설명은 하위 폴더의 README 링크를 통해 자세히 보실 수 있습니다:

* [🔥 화재 감지 알고리즘 및 YOLO 모델 설명](./src/urdf_tuto/fire_detect/README.md)
* [🛸 드론 감지 알고리즘 및 노드 설정 가이드](./src/urdf_tuto/drone_detector/README.md)
* [🛰️ 3D 라이다-카메라 센서 융합 및 매핑 프로세스 설명](./src/urdf_tuto/pointcloud_process/README.md)

