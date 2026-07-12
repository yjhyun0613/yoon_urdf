import open3d as o3d
import numpy as np
import os
import sys

obj_path = "/home/yoon/yoon_urdf/src/urdf_tuto/models/gangnam_world/meshes/gangnam_sector_b.obj"
output_dir = "/home/yoon/yoon_urdf"
top_img = os.path.join(output_dir, "gangnam_render_top.png")
iso_img = os.path.join(output_dir, "gangnam_render_iso.png")

print("Loading OBJ file for rendering...")
mesh = o3d.io.read_triangle_mesh(obj_path)
if mesh.is_empty():
    print("Error: Mesh is empty.")
    sys.exit(1)

# 법선 벡터 계산 (음영 렌더링을 위함)
mesh.compute_vertex_normals()

# 1. 탑뷰 (Top-down view) 렌더링
print("Rendering Top-down view...")
vis = o3d.visualization.Visualizer()
vis.create_window(window_name="Top View", width=1280, height=960, visible=False)
vis.add_geometry(mesh)

# Bounding box 기준 카메라 시점 자동 맞춤 (검은 화면 방지)
vis.reset_view_point(True)

ctr = vis.get_view_control()
ctr.set_front([0, -1, 0]) # Y-up 모델 기준 위에서 아래로 조준
ctr.set_up([0, 0, 1])

vis.poll_events()
vis.update_renderer()
vis.capture_screen_image(top_img)
vis.destroy_window()
print(f"Top view saved to {top_img}")

# 2. 사선뷰 (Isometric / Perspective view) 렌더링
print("Rendering Perspective view...")
vis = o3d.visualization.Visualizer()
vis.create_window(window_name="Perspective View", width=1280, height=960, visible=False)
vis.add_geometry(mesh)

# Bounding box 기준 카메라 시점 자동 맞춤
vis.reset_view_point(True)

ctr = vis.get_view_control()
# 카메라 각도 회전 적용해 사선 뷰 생성
ctr.rotate(350.0, 200.0)

vis.poll_events()
vis.update_renderer()
vis.capture_screen_image(iso_img)
vis.destroy_window()
print(f"Perspective view saved to {iso_img}")

print("Rendering process successfully completed!")
