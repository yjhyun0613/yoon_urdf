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
        
        # Calibration World XML (Only AMR robot and 4 cardinal checkerboard targets, no humanoid, no obstacles)
        self.xml_string = f"""
        <mujoco model="calibration_simulation">
          <asset>
            <texture name="checkerboard" type="2d" builtin="checker" width="512" height="512" rgb1=".1 .1 .1" rgb2=".9 .9 .9"/>
            <material name="mat_checker" texture="checkerboard" texrepeat="4 3"/>
          </asset>

          <worldbody>
            <light name="top_light" pos="0 0 4" dir="0 0 -1" castshadow="false"/>
            <geom name="floor" type="plane" size="10 10 0.1" rgba="0.8 0.8 0.8 1"/>

            <!-- 3D Checkerboards and white backing boards for camera calibration (8x6 squares -> 7x5 inner corners) -->
            <!-- Placed in 4 directions, facing oncoming camera perpendicularly for direct high-quality detection -->
            <geom name="calibration_backing_1" type="box" size="0.01 0.45 0.35" pos="1.309 0.005 0.8" euler="0 10 -152.5" rgba="1 1 1 1"/>
            <geom name="calibration_board_1" type="box" size="0.002 0.38 0.28" pos="1.3 0.0 0.8" euler="0 10 -152.5" material="mat_checker"/>
            
            <geom name="calibration_backing_2" type="box" size="0.01 0.45 0.35" pos="-0.005 1.309 0.8" euler="0 10 -62.5" rgba="1 1 1 1"/>
            <geom name="calibration_board_2" type="box" size="0.002 0.38 0.28" pos="0.0 1.3 0.8" euler="0 10 -62.5" material="mat_checker"/>
            
            <geom name="calibration_backing_3" type="box" size="0.01 0.45 0.35" pos="-1.309 -0.005 0.8" euler="0 10 27.5" rgba="1 1 1 1"/>
            <geom name="calibration_board_3" type="box" size="0.002 0.38 0.28" pos="-1.3 0.0 0.8" euler="0 10 27.5" material="mat_checker"/>
            
            <geom name="calibration_backing_4" type="box" size="0.01 0.45 0.35" pos="0.005 -1.309 0.8" euler="0 10 117.5" rgba="1 1 1 1"/>
            <geom name="calibration_board_4" type="box" size="0.002 0.38 0.28" pos="0.0 -1.3 0.8" euler="0 10 117.5" material="mat_checker"/>

            <!-- AMR robot body -->
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
        
        self.get_logger().info(f"Initializing Isolated MuJoCo Calibration Scene with fovy={self.fovy}...")
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
        self.get_logger().info("Isolated Calibration Simulator initialized.")

    def timer_callback(self):
        t = self.get_clock().now().nanoseconds * 1e-9
        
        # 1. Make the AMR move in a circular trajectory of radius 0.6m
        radius = 0.6
        omega = 0.3
        x_amr = radius * np.cos(omega * t)
        y_amr = radius * np.sin(omega * t)
        z_amr = 0.1
        
        heading = omega * t + np.pi / 2.0
        
        # AMR freejoint starts at qpos[0:7] (no humanoid in this model)
        self.data.qpos[0] = x_amr
        self.data.qpos[1] = y_amr
        self.data.qpos[2] = z_amr
        
        # Quaternion for yaw rotation (heading) around Z-axis
        self.data.qpos[3] = np.cos(heading / 2.0)  # w
        self.data.qpos[4] = 0.0                     # x
        self.data.qpos[5] = 0.0                     # y
        self.data.qpos[6] = np.sin(heading / 2.0)  # z
        
        self.data.qvel[:] = 0.0
        
        mujoco.mj_step(self.model, self.data)
        
        if self.viewer.is_running():
            self.viewer.sync()
        
        # Render onboard camera
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
        
        # Publish AMR Pose
        pose_msg = PoseStamped()
        pose_msg.header.stamp = now_msg
        pose_msg.header.frame_id = "map"
        pose_msg.pose.position.x = x_amr
        pose_msg.pose.position.y = y_amr
        pose_msg.pose.position.z = z_amr
        pose_msg.pose.orientation.w = self.data.qpos[3]
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = self.data.qpos[6]
        self.pose_pub.publish(pose_msg)
        
        # Show view GUI
        cv2.imshow("Calibration Onboard Camera View", bgr_img)
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
