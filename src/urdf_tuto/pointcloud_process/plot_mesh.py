import xml.etree.ElementTree as ET
import numpy as np
import matplotlib
matplotlib.use('Agg') # GUI 윈도우 팝업 비활성화 (Headless 세팅)
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import sys

dae_path = "/home/yoon/yoon_urdf/src/urdf_tuto/models/gangnam_world/meshes/gangnam_sector_b.dae"
output_image = "/home/yoon/yoon_urdf/gangnam_matplotlib_view.png"

print("Parsing DAE file to extract points for matplotlib...")
try:
    context = ET.iterparse(dae_path, events=('end',))
    points = []
    count = 0
    
    for event, elem in context:
        tag_local = elem.tag.split('}')[-1] if '}' in elem.tag else elem.tag
        if tag_local == 'float_array':
            elem_id = elem.get('id', '').lower()
            if 'pos' in elem_id or 'mesh' in elem_id:
                coords = np.fromstring(elem.text, sep=' ')
                if len(coords) >= 3:
                    pts = coords.reshape(-1, 3)
                    points.append(pts)
                    count += len(pts)
                    if count > 100000: # 10만개 샘플링
                        break
        elem.clear()
        
    if len(points) == 0:
        print("Fallback: extracting float arrays...")
        context = ET.iterparse(dae_path, events=('end',))
        for event, elem in context:
            tag_local = elem.tag.split('}')[-1] if '}' in elem.tag else elem.tag
            if tag_local == 'float_array':
                coords = np.fromstring(elem.text, sep=' ')
                if len(coords) >= 900 and len(coords) % 3 == 0:
                    pts = coords.reshape(-1, 3)
                    points.append(pts)
                    count += len(pts)
                    if count > 100000:
                        break
            elem.clear()

    if len(points) > 0:
        all_pts = np.vstack(points)
        print(f"Sampled {len(all_pts)} raw vertices. Generating matplotlib 3D scatter plot...")
        
        # 속도를 높이고 가독성을 확보하기 위해 10,000개만 무작위 추출
        sample_size = min(15000, len(all_pts))
        indices = np.random.choice(len(all_pts), sample_size, replace=False)
        sampled_pts = all_pts[indices]
        
        # mm -> m 환산
        X = sampled_pts[:, 0] * 0.001
        Y = -sampled_pts[:, 2] * 0.001 # Z-up 기준 변환
        Z = sampled_pts[:, 1] * 0.001  # Y가 높이축이었으므로 Z축으로 대입
        
        # Matplotlib 3D 산점도 생성
        fig = plt.figure(figsize=(12, 10))
        ax = fig.add_subplot(111, projection='3d')
        
        # 높이(Z)값에 따라 색상을 다르게 칠함
        sc = ax.scatter(X, Y, Z, c=Z, cmap='plasma', s=0.8, alpha=0.8)
        
        ax.set_title("Mobiltech Gangnam Sector B (Raw Scaled Point-Cloud View)", fontsize=14)
        ax.set_xlabel("X (Meters)")
        ax.set_ylabel("Y (Meters)")
        ax.set_zlabel("Height Z (Meters)")
        
        # 축 비율 맞추기 (왜곡 방지)
        max_range = np.array([X.max()-X.min(), Y.max()-Y.min(), Z.max()-Z.min()]).max() / 2.0
        mid_x = (X.max()+X.min()) * 0.5
        mid_y = (Y.max()+Y.min()) * 0.5
        mid_z = (Z.max()+Z.min()) * 0.5
        ax.set_xlim(mid_x - max_range, mid_x + max_range)
        ax.set_ylim(mid_y - max_range, mid_y + max_range)
        ax.set_zlim(mid_z - max_range, mid_z + max_range)
        
        fig.colorbar(sc, ax=ax, label='Height (m)', shrink=0.6)
        
        # 최적의 사선 뷰 각도 설정
        ax.view_init(elev=35, azim=45)
        
        plt.savefig(output_image, dpi=200, bbox_inches='tight')
        print(f"PLOT_SAVED: {output_image}")
    else:
        print("Error: No vertices extracted.")
except Exception as e:
    print(f"Error: {e}")
