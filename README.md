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

### 5. 3D 컬러 포인트클라우드 매핑 실행
```bash
ros2 launch urdf_tuto launch_sim_3d.launch.py
```
* **Lidar-Camera Fusion**: 3D 라이다 포인트들을 실시간으로 카메라 이미지 프레임 상에 투영해 색상(RGB) 정보를 결합합니다.
* **보셀 맵 누적**: 수집된 3D 컬러 포인트들을 5mm 단위 보셀 그리드로 누적하여 완성도 높은 3D 지도를 구성합니다.
* **자동 파일 저장**: 누적된 3D 지도는 시뮬레이션 실행 도중 15초 마다, 그리고 시뮬레이션 종료 시점에 `/home/yoon/yoon_urdf/saved_maps/` 폴더에 `.ply` 파일 포맷으로 자동 저장됩니다.

#### 💡 저장된 3D 맵 (.ply) 시각화 확인 방법
파이썬 `open3d` 라이브러리를 통해 터미널에서 즉시 3D 창을 열어 마우스로 지도를 돌려보며 관찰할 수 있습니다:
```bash
python3 -c "import open3d as o3d; pcd = o3d.io.read_point_cloud('saved_maps/<저장된_파일_이름>.ply'); o3d.visualization.draw_geometries([pcd])"
```
*(또는 `MeshLab`이나 `CloudCompare` 같은 GUI 전문 시뮬레이션 툴을 설치하여 열어볼 수 있습니다.)*

---

## 📂 상세 기능 가이드 (하위 문서 링크)

이 프로젝트의 상세 구성 및 각 노드별 설명은 하위 폴더의 README 링크를 통해 자세히 보실 수 있습니다:

* [🔥 화재 감지 알고리즘 및 YOLO 모델 설명](./src/urdf_tuto/fire_detect/README.md)
* [🛸 드론 감지 알고리즘 및 노드 설정 가이드](./src/urdf_tuto/drone_detector/README.md)
* [🛰️ 3D 라이다-카메라 센서 융합 및 매핑 프로세스 설명](./src/urdf_tuto/pointcloud_process/README.md)
* [📊 3D 라이다 센서 하드웨어 스펙 비교](./src/urdf_tuto/resource/3d_lidar_specs/README.md)

