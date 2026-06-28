# AMR Onboard Vision 3D Mapping Scenario: Feature Point Triangulation

This document defines the simulation scenario for verifying the real-time feature-point (Visual SLAM style) 3D reconstruction and mapping system.

---

## 1. Core Role & Responsibility (특징점 기반 3D 복원)
우리는 **움직이는 AMR의 눈**으로서 주변 공간의 특징점(Feature Points)들을 추출하고 매칭하여 지도를 실시간으로 기하학적으로 복원합니다.
* **특징점 추출**: 이미지 내 코너, 질감 변화 지점을 `ORB 특징점 검출기`로 실시간 검출합니다.
* **실시간 매칭 및 추적**: 로봇이 주행함에 따라 이전 프레임과 현재 프레임 사이의 특징점들을 매칭합니다.
* **3D 삼각측량 (Triangulation)**: 로봇의 이동 거리(오도메트리)와 매칭 쌍들의 시차(Parallax)를 결합하여 공간 상의 절대 3D 좌표 $(X, Y, Z)$를 계산합니다.
* **데이터 병합 및 전송**: 복원된 3D 점군 데이터를 `sensor_msgs/msg/PointCloud2`와 `nav_msgs/msg/OccupancyGrid` 형태로 외부로 실시간 전송합니다.

---

## 2. 시뮬레이션 환경 구성
* **공간**: 10m x 10m 크기의 공간. 특징점 검출이 용이하도록 바닥에는 체커보드 패턴이 활성화되어 있으며, 주변에 여러 장애물 물체들이 배치되어 있습니다.
* **AMR 주행**: 가로/세로 영역을 가로지르는 8자형(Figure-8) 자율 경로를 추종하며 회전하며 전방을 카메라로 스캔합니다.
  * $x(t) = 2.5 \sin(0.15 t)$
  * $y(t) = 1.2 \cos(0.30 t)$

---

## 3. 시나리오 실행 단계 (Run-Time Stages)

| 단계 | AMR 상태 | 비전 처리 (눈) | 매핑 및 전송 상태 |
| :--- | :--- | :--- | :--- |
| **1. 시작 단계** | `(0.0, 1.2)` 부근에서 주행 시작. | 첫 프레임의 ORB 특징점 추출. | 빈 점군(Point Cloud) 및 지도 전송. |
| **2. 이동 및 매칭** | 전방으로 이동하며 카메라 주행. | 이전 프레임과 매칭 쌍 매주 확보. | 삼각측량을 통해 특징점들의 3D 절대 좌표 연산 시작. |
| **3. 3D 점군 지도화** | 8자 회전 기동 수행. | 시차가 커짐에 따라 고정 정밀도가 높은 3D 포인트 대량 복원. | 복원된 포인트들을 `PointCloud2` 토픽으로 내보내 RViz2에 표시 및 격자 지도에 누적. |
| **4. 지도 완성** | 8자 주행을 무한 반복. | 방안 구석구석 특징점들 상시 갱신. | 실시간 완성된 방 안의 3D 공간 기하 점군 지도가 다른 시스템으로 지속 전송됨. |
