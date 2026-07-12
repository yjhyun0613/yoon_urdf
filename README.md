# ROS2 URDF 연습 프로젝트 (yoon_urdf)


이 저장소는 ROS2를 기반으로 URDF 로봇 모델링, Gazebo 시뮬레이션 환경 구축, 그리고 YOLO를 이용한 객체 탐지 기능을 실습하기 위해 만들어졌습니다.

---

## 🛠️ 공통 빌드 및 환경 설정 방법

시뮬레이션을 구동하기 전, 먼저 워크스페이스를 빌드하고 환경 설정을 적용해야 합니다:

```bash
# 1. 워크스페이스 이동
cd ~/yoon_urdf

# 2. 패키지 빌드
colcon build --symlink-install

# 3. 환경 설정 적용
source install/setup.bash
```

---

## 📂 프로젝트 상세 기능 가이드 (하위 문서 링크)

이 프로젝트는 다음과 같은 하위 시뮬레이션 기능들로 구성되어 있습니다. 각 런치 파일의 구체적인 실행 방법 및 상세 설계는 아래 링크의 상세 문서를 확인해 주세요:

| 주요 시뮬레이션 기능 | 관련 런치 파일 | 상세 가이드 문서 링크 |
| :--- | :--- | :--- |
| **🤖 기본 로봇 시뮬레이션** | `launch_sim.launch.py` | [로봇 URDF 모델 및 기본 시뮬레이션 설명](./src/urdf_tuto/urdf/README.md) |
| **🔥 불 감지 시뮬레이션** | `fire_detect.launch.py` | [화재 감지 알고리즘 및 YOLO 모델 설명](./src/urdf_tuto/fire_detect/README.md) |
| **🛸 드론 감지 시뮬레이션** | `drone_detect.launch.py` | [드론 감지 알고리즘 및 노드 설정 가이드](./src/urdf_tuto/drone_detector/README.md) |
| **🛰️ 3D 컬러 매핑 시뮬레이션** | `launch_sim_3d.launch.py` | [3D 라이다-카메라 센서 융합 및 매핑 가이드](./src/urdf_tuto/pointcloud_process/README.md) |
| **🏙️ 강남 3D 컬러 매핑 시뮬레이션** | `launch_sim_gangnam.launch.py` | [3D 라이다-카메라 센서 융합 및 매핑 가이드](./src/urdf_tuto/pointcloud_process/README.md) |


