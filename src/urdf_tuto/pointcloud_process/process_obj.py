import open3d as o3d
import numpy as np
import sys

obj_path = "/home/yoon/yoon_urdf/src/urdf_tuto/models/gangnam_world/meshes/gangnam_sector_b.obj"
output_path = "/home/yoon/yoon_urdf/src/urdf_tuto/models/gangnam_world/meshes/gangnam_sector_b_fixed.obj"

print("Loading OBJ file using Open3D...")
mesh = o3d.io.read_triangle_mesh(obj_path)

if mesh.is_empty():
    print("Failed to load mesh.")
    sys.exit(1)

vertices = np.asarray(mesh.vertices)
print(f"Loaded {len(vertices)} vertices.")

# 1. 스케일 축소 (mm -> m)
vertices = vertices * 0.001

# 2. Y-up (Y가 높이) -> Z-up (Z가 높이) 축 변환
# 보통 오른손 좌표계 변환: X_new = X, Y_new = -Z, Z_new = Y
fixed_vertices = np.zeros_like(vertices)
fixed_vertices[:, 0] = vertices[:, 0]
fixed_vertices[:, 1] = -vertices[:, 2]
fixed_vertices[:, 2] = vertices[:, 1]

# 3. 중심점을 XY평면 원점(0,0)으로 이동
mean = np.mean(fixed_vertices, axis=0)
print(f"Computed Centroid in Meters: {mean}")
fixed_vertices[:, 0] -= mean[0]
fixed_vertices[:, 1] -= mean[1]

# Z축(높이)은 맵의 가장 최하단 바닥면이 Z=0에 딱 오도록 정렬 (로봇 주행을 위함)
min_z = np.min(fixed_vertices[:, 2])
fixed_vertices[:, 2] -= min_z
print(f"New Z-range (height): {np.min(fixed_vertices[:, 2]):.4f}m ~ {np.max(fixed_vertices[:, 2]):.4f}m")
print(f"New X-range: {np.min(fixed_vertices[:, 0]):.4f}m ~ {np.max(fixed_vertices[:, 0]):.4f}m")
print(f"New Y-range: {np.min(fixed_vertices[:, 1]):.4f}m ~ {np.max(fixed_vertices[:, 1]):.4f}m")

mesh.vertices = o3d.utility.Vector3dVector(fixed_vertices)

print("Writing processed OBJ file...")
o3d.io.write_triangle_mesh(output_path, mesh)
print("OBJ processing successfully finished!")
