#!/usr/bin/env python3
import mujoco

xml_path = "/home/yoon/yoon_urdf/src/yoon_urdf/yoon_urdf/mujoco_cam_publisher.py"

# Read the xml_string from the file
with open(xml_path, 'r') as f:
    content = f.read()

# Extract self.xml_string
start_idx = content.find('self.xml_string = f"""')
if start_idx == -1:
    start_idx = content.find('self.xml_string = """')
start_idx = content.find('"""', start_idx) + 3
end_idx = content.find('"""', start_idx)
xml_string = content[start_idx:end_idx].strip()

# Format any placeholders like {self.fovy}
xml_string = xml_string.replace('{self.fovy}', '60.0')

try:
    model = mujoco.MjModel.from_xml_string(xml_string)
    print("\nSUCCESSfully loaded XML string in MuJoCo!")
    print(f"Total geoms: {model.ngeom}")
    for i in range(model.ngeom):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i)
        pos = model.geom_pos[i]
        size = model.geom_size[i]
        print(f" - Geom #{i}: Name='{name}', Pos={pos.tolist()}, Size={size.tolist()}")
except Exception as e:
    print(f"ERROR: {e}")
