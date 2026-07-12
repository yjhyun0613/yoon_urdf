import xml.etree.ElementTree as ET
import numpy as np
import sys

dae_path = "/home/yoon/yoon_urdf/src/urdf_tuto/models/gangnam_world/meshes/gangnam_sector_b.dae"

print("Parsing XML and extracting vertices...")
try:
    context = ET.iterparse(dae_path, events=('end',))
    points = []
    count = 0
    
    for event, elem in context:
        # float_array 노드 중 id에 position 또는 pos가 들어간 것을 우선 탐색
        tag_local = elem.tag.split('}')[-1] if '}' in elem.tag else elem.tag
        if tag_local == 'float_array':
            elem_id = elem.get('id', '').lower()
            if 'pos' in elem_id or 'mesh' in elem_id:
                coords = np.fromstring(elem.text, sep=' ')
                if len(coords) >= 3:
                    pts = coords.reshape(-1, 3)
                    points.append(pts)
                    count += len(pts)
                    if count > 1000000: # 100만개 샘플링
                        break
        elem.clear()
        
    if len(points) == 0:
        print("Fallback: extracting all float arrays with length multiple of 3...")
        context = ET.iterparse(dae_path, events=('end',))
        for event, elem in context:
            tag_local = elem.tag.split('}')[-1] if '}' in elem.tag else elem.tag
            if tag_local == 'float_array':
                coords = np.fromstring(elem.text, sep=' ')
                if len(coords) >= 900 and len(coords) % 3 == 0: # 유의미한 크기의 메쉬 데이터만
                    pts = coords.reshape(-1, 3)
                    points.append(pts)
                    count += len(pts)
                    if count > 1000000:
                        break
            elem.clear()

    if len(points) > 0:
        all_pts = np.vstack(points)
        mean = np.mean(all_pts, axis=0)
        std = np.std(all_pts, axis=0)
        mins = np.min(all_pts, axis=0)
        maxs = np.max(all_pts, axis=0)
        print(f"ANALYSIS_SUCCESS")
        print(f"Mean (Center of mass): {mean[0]:.6f}, {mean[1]:.6f}, {mean[2]:.6f}")
        print(f"Std deviation: {std[0]:.6f}, {std[1]:.6f}, {std[2]:.6f}")
        print(f"Min bound: {mins[0]:.6f}, {mins[1]:.6f}, {mins[2]:.6f}")
        print(f"Max bound: {maxs[0]:.6f}, {maxs[1]:.6f}, {maxs[2]:.6f}")
    else:
        print("Error: No vertices found in DAE file.")
except Exception as e:
    print(f"Error during analysis: {e}")
