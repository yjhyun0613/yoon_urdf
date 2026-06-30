#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import mujoco
import mujoco.viewer
import cv2
import numpy as np
import math

class MujocoCalibrationPublisher(Node):
    def __init__(self):
        super().__init__('mujoco_calibration_publisher')
        
        # ROS2 Publishers
        self.img_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/camera/camera_info', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/robot_global_pose', 10)
        
        self.declare_parameter('fovy', 60.0)
        self.fovy = self.get_parameter('fovy').value
        
        # Calibration World XML with a single mocap-controlled calibration board
        self.xml_string = f"""
        <mujoco model="calibration_simulation">
          <asset>
            <texture name="checkerboard" type="2d" file="/home/yoon/yoon_urdf/src/yoon_urdf/resource/checkerboard.png"/>
            <material name="mat_checker" texture="checkerboard"/>
          </asset>

          <worldbody>
            <light name="top_light" pos="0 0 4" dir="0 0 -1" castshadow="false"/>
            <geom name="floor" type="plane" size="10 10 0.1" rgba="0.8 0.8 0.8 1"/>

            <!-- Mocap-controlled calibration board -->
            <body name="calibration_board" mocap="true">
              <geom name="calibration_backing" type="box" size="0.01 0.45 0.35" rgba="1 1 1 1"/>
              <!-- Place the chessboard texture board 1.1cm in front of the backing board to prevent z-fighting -->
              <geom name="calibration_board" type="box" size="0.002 0.38 0.28" pos="0.011 0 0" material="mat_checker"/>
            </body>

            <!-- AMR robot body (Static at origin, camera looking forward +X) -->
            <body name="amr" pos="0 0 0.1">
              <freejoint name="amr_root"/>
              <!-- Base (Dark grey cylinder) -->
              <geom name="amr_base" type="cylinder" size="0.25 0.1" rgba="0.15 0.15 0.15 1"/>
              <!-- Wheels -->
              <geom name="amr_wheel_l" type="cylinder" size="0.08 0.02" pos="0 0.24 0" euler="90 0 0" rgba="0.1 0.1 0.1 1"/>
              <geom name="amr_wheel_r" type="cylinder" size="0.08 0.02" pos="0 -0.24 0" euler="90 0 0" rgba="0.1 0.1 0.1 1"/>
              <!-- Main vertical support pole -->
              <geom name="amr_pole" type="cylinder" size="0.03 0.5" pos="0 0.5 0" rgba="0.75 0.75 0.75 1"/>
              <!-- Trays -->
              <geom name="amr_tray1" type="cylinder" size="0.22 0.01" pos="0 0 0.3" rgba="0.9 0.9 0.9 1"/>
              <geom name="amr_tray2" type="cylinder" size="0.22 0.01" pos="0 0 0.6" rgba="0.9 0.9 0.9 1"/>
              <geom name="amr_tray3" type="cylinder" size="0.22 0.01" pos="0 0 0.9" rgba="0.9 0.9 0.9 1"/>
              <!-- Onboard camera mounted on the AMR, looking forward (+X direction of the AMR body) -->
              <camera name="onboard_camera" pos="0.25 0 0.8" xyaxes="0 -1 0 0 0 1" fovy="{self.fovy}"/>
            </body>
          </worldbody>
        </mujoco>
        """
        
        self.get_logger().info(f"Initializing Mocap Calibration Scene with fovy={self.fovy}...")
        self.model = mujoco.MjModel.from_xml_string(self.xml_string)
        self.data = mujoco.MjData(self.model)
        
        # Initialize Renderer (640x480 resolution)
        self.width = 640
        self.height = 480
        self.renderer = mujoco.Renderer(self.model, height=self.height, width=self.width)
        
        self.bridge = CvBridge()
        
        # Start passive 3D viewer
        self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
        
        # Timer for simulation step (20 Hz)
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.get_logger().info("Mocap Calibration Simulator initialized.")

    def euler_to_quat(self, roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        return [qw, qx, qy, qz]

    def timer_callback(self):
        t = self.get_clock().now().nanoseconds * 1e-9
        
        # 1. Keep the AMR robot static at the origin facing +X
        self.data.qpos[0] = 0.0
        self.data.qpos[1] = 0.0
        self.data.qpos[2] = 0.1
        self.data.qpos[3] = 1.0  # qw
        self.data.qpos[4] = 0.0  # qx
        self.data.qpos[5] = 0.0  # qy
        self.data.qpos[6] = 0.0  # qz
        self.data.qvel[:] = 0.0
        
        # 2. Animate the mocap calibration board to mimic a human waving it
        # Camera is at absolute x=0.25, y=0.0, z=0.9
        # Distance (X): Oscillate slowly between 0.6m and 1.1m from camera
        d = 0.85 + 0.25 * math.sin(0.4 * t)
        x_board = 0.25 + d
        
        # Horizontal (Y): Oscillate left/right to sweep boundary distortion
        y_board = 0.3 * math.sin(0.6 * t)
        
        # Vertical (Z): Oscillate up/down around the camera height (0.9m)
        z_board = 0.9 + 0.2 * math.cos(0.5 * t)
        
        # Angles (Roll/Pitch/Yaw): Tilt the board for perspective diversity
        roll = 0.3 * math.sin(0.7 * t)
        pitch = 0.3 * math.cos(0.5 * t)
        # Board faces camera, so yaw is around 180 degrees (pi rad)
        yaw = math.pi + 0.3 * math.sin(0.3 * t)
        
        qw, qx, qy, qz = self.euler_to_quat(roll, pitch, yaw)
        
        # Write mocap pose to MuJoCo
        mocap_id = self.model.body_mocapid[mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "calibration_board")]
        self.data.mocap_pos[mocap_id] = [x_board, y_board, z_board]
        self.data.mocap_quat[mocap_id] = [qw, qx, qy, qz]
        
        # Step simulation
        mujoco.mj_step(self.model, self.data)
        
        if self.viewer.is_running():
            self.viewer.sync()
        
        # Render onboard camera view
        self.renderer.update_scene(self.data, camera="onboard_camera")
        rgb_img = self.renderer.render()
        bgr_img = cv2.cvtColor(rgb_img, cv2.COLOR_RGB2BGR)
        
        now_msg = self.get_clock().now().to_msg()
        
        # Publish Image
        img_msg = self.bridge.cv2_to_imgmsg(bgr_img, encoding="bgr8")
        img_msg.header.stamp = now_msg
        img_msg.header.frame_id = "camera_link"
        self.img_pub.publish(img_msg)
        
        # Publish CameraInfo
        info_msg = CameraInfo()
        info_msg.header.stamp = now_msg
        info_msg.header.frame_id = "camera_link"
        info_msg.width = self.width
        info_msg.height = self.height
        
        fovy_rad = math.radians(self.fovy)
        fy = self.height / (2.0 * math.tan(fovy_rad / 2.0))
        fx = fy
        cx, cy = 320.0, 240.0
        info_msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info_msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info_msg.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.info_pub.publish(info_msg)
        
        # Publish static AMR Pose
        pose_msg = PoseStamped()
        pose_msg.header.stamp = now_msg
        pose_msg.header.frame_id = "map"
        pose_msg.pose.position.x = 0.0
        pose_msg.pose.position.y = 0.0
        pose_msg.pose.position.z = 0.1
        pose_msg.pose.orientation.w = 1.0
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = 0.0
        self.pose_pub.publish(pose_msg)
        
        # Show view GUI
        cv2.imshow("Mocap Calibration Onboard Camera View", bgr_img)
        cv2.waitKey(1)

    def destroy_node(self):
        if hasattr(self, 'viewer') and self.viewer.is_running():
            self.viewer.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MujocoCalibrationPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
