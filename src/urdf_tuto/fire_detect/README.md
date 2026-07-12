# 🔥 화재 감지 모듈 (fire_detect)

이 폴더는 로봇의 카메라 입력을 활용하여 실시간으로 화재 및 연기를 감지하는 YOLO 기반 인지 모듈을 포함하고 있습니다.

## 📁 주요 구성 파일
* `fire_detector_01.py`: 카메라 이미지를 받아 YOLO 모델 추론을 돌리고, 화재 여부를 판단하는 ROS 2 노드입니다.
* `fire_detect.pt` / `fire_detect1.pt`: 화재/연기 탐지를 위해 학습된 YOLOv8 가중치(Weights) 파일입니다.

## 🚀 실행 방법
아래 명령어를 사용하여 가제보 환경 및 불 감지 노드를 함께 실행할 수 있습니다:
```bash
ros2 launch urdf_tuto fire_detect.launch.py
```

## 🛠️ 기능 특징
* **실시간 객체 탐지**: Gazebo 상의 화재 오브젝트를 감지하여 바운딩 박스를 칩니다.
* **시뮬레이션 시간 동기화**: `use_sim_time:=true` 매개변수를 활성화하여 시뮬레이션 타임 스탬프와 데이터 속도를 완벽히 일치시킵니다.
