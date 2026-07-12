# 🛰️ 3D 포인트클라우드 및 센서 융합 모듈 (pointcloud_process)

이 폴더는 3D 라이다와 카메라의 원시 데이터를 받아 실시간으로 센서 퓨전을 수행하고, 색상이 입혀진 컬러 3D 지도를 실시간으로 구축 및 저장하는 패키지 핵심 알고리즘이 모여 있는 곳입니다.

## 📁 주요 구성 파일
* `lidar_camera_fusion.py`:
  * 3D 라이다에서 나온 `PointCloud2` 데이터의 점들을 카메라 내부 파라미터를 이용해 2D 이미지 좌표계 상에 실시간 투영(Projection)합니다.
  * 투영된 픽셀 위치의 RGB 색상값을 얻어 각 라이다 포인트에 입혀진 컬러 포인트클라우드 토픽(`/semantic_pointcloud`)을 발행합니다.
* `pointcloud_accumulator.py`:
  * 발행된 `/semantic_pointcloud` 토픽을 받아 **보셀 그리드(Voxel Grid)** 형태로 촘촘하게 데이터를 실시간으로 누적합니다.
  * 누적된 지도는 15초 주기 및 시뮬레이션 종료 시점에 `/home/yoon/yoon_urdf/saved_maps/` 폴더에 `.ply` 파일로 자동 저장됩니다.

## 🚀 실행 방법
가제보 상에서 실행하려는 3D 환경 맵에 따라 아래 명령어 중 하나를 선택해 시뮬레이션을 기동합니다:

* **오피스 환경 (cpr_office) 매핑**:
  ```bash
  ros2 launch urdf_tuto launch_sim_3d.launch.py
  ```
* **모빌테크 실외 강남 환경 (gangnam_world) 매핑**:
  ```bash
  ros2 launch urdf_tuto launch_sim_gangnam.launch.py
  ```

## 🛠️ 주요 파라미터 조정
* **보셀 해상도 (`voxel_size`)**: 맵이 누적되는 점 간격의 세밀함을 정의합니다. 현재 5mm(`0.005m`)로 설정되어 있어 고해상도 매핑이 가능합니다. (`launch_sim_3d.launch.py`에서 변경 가능)
* **저장 주기 (`save_interval_sec`)**: 자동 매핑 저장 주기를 조정합니다. 기본값은 15초입니다.

## 💡 저장된 3D 맵 (.ply) 시각화 확인 방법
파이썬 `open3d` 라이브러리를 통해 터미널에서 즉시 3D 창을 열어 마우스로 지도를 돌려보며 관찰할 수 있습니다:
```bash
python3 -c "import open3d as o3d; pcd = o3d.io.read_point_cloud('/home/yoon/yoon_urdf/saved_maps/<저장된_파일_이름>.ply'); o3d.visualization.draw_geometries([pcd])"
```
*(또는 `MeshLab`이나 `CloudCompare` 같은 GUI 전문 시뮬레이션 툴을 설치하여 열어볼 수 있습니다.)*

---

## 📊 센서 하드웨어 참고 자료
실제 상용 3D LiDAR 센서들(Velodyne, Ouster, Livox, RoboSense)의 상세 사양은 아래 문서에서 확인하실 수 있습니다:
* [실제 3D LiDAR 센서 스펙 정리 문서](../resource/3d_lidar_specs/README.md)

