# 🤖 로봇 URDF 모델 및 기본 시뮬레이션 (urdf)

이 폴더는 로봇의 3D 형상 모델링(URDF/Xacro) 정의 파일들과 센서/액추에이터 시뮬레이션 플러그인이 모여 있는 곳입니다.

## 📁 주요 구성 파일
* `robot_3.xacro`: 차륜형 로봇 본체, 2D 라이다, 온보드 카메라가 장착된 기본 로봇 모델 정의 파일입니다.
* `macro.xacro`: 관성 모멘트(Inertia) 계산을 위한 수학적 매크로 함수들이 모여 있습니다.
* `gazebo.xacro`: 차동 구동(Differential Drive)을 위한 가제보 제어용 플러그인이 포함되어 있습니다.

## 🚀 실행 방법
아래 명령어를 사용하여 기본 로봇 모델링 시뮬레이션 및 RViz 시각화를 구동합니다:
```bash
ros2 launch urdf_tuto launch_sim.launch.py
```
* 로봇 조종을 위해 다른 터미널에서 키보드 제어 노드를 띄울 수 있습니다:
  ```bash
  ros2 run teleop_twist_keyboard teleop_twist_keyboard
  ```

## 🛠️ 포함 센서 스펙
* **카메라**: 640x480 해상도 온보드 카메라 (`/my_robot/camera/image_raw`)
* **2D 라이다**: 360도 전방위 감지 2D 라이다 (`/scan`)
