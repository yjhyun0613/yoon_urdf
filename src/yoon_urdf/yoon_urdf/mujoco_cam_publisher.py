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
import yaml
import os

import math

class MujocoCamPublisher(Node):
    def __init__(self):
        super().__init__('mujoco_cam_publisher')
        
        # ROS2 Publishers
        self.img_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/camera/camera_info', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/robot_global_pose', 10)
        
        # Declare fovy parameter (default 90 degrees for standard perspective lens)
        self.declare_parameter('fovy', 90.0)
        self.fovy = self.get_parameter('fovy').value
        
        # Load camera calibration for undistortion display
        self.camera_matrix = None
        self.distortion_coeffs = None
        yaml_path = '/home/yoon/yoon_urdf/auto_camera_calibration.yaml'
        if os.path.exists(yaml_path):
            try:
                with open(yaml_path, 'r') as f:
                    calib_data = yaml.safe_load(f)
                self.camera_matrix = np.array(calib_data['camera_matrix']['data']).reshape((3, 3))
                self.distortion_coeffs = np.array(calib_data['distortion_coefficients']['data'])
                self.get_logger().info(f'Loaded calibration from {yaml_path} for undistorted display')
            except Exception as e:
                self.get_logger().warn(f'Failed to load calibration: {e}')
        
        # Integrated scene XML: Include humanoid.xml, and add AMR body in worldbody
        # Place three distinct pedestrian cylinder targets at different locations
        self.xml_string = f"""
        <mujoco model="integrated_simulation">
          <include file="/home/yoon/.mujoco/mujoco-3.9.0/model/humanoid/humanoid.xml"/>

          <asset>
          </asset>

          <worldbody>

            <!-- Three colored cylinder target obstacles representing pedestrians in the hall -->
            <!-- Size: radius=0.2m, half-height=0.85m (full height = 1.7m) -->
            <geom name="target_a" type="cylinder" size="0.2 0.85" pos="1.5 0.5 0.85" rgba="1.0 0.0 0.0 1"/>
            <geom name="target_b" type="cylinder" size="0.2 0.85" pos="-1.5 -0.5 0.85" rgba="0.0 1.0 0.0 1"/>
            <geom name="target_c" type="cylinder" size="0.2 0.85" pos="0.0 1.2 0.85" rgba="0.0 0.0 1.0 1"/>

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
        
        self.get_logger().info(f"Initializing MuJoCo Integrated Scene with fovy={self.fovy}...")
        self.model = mujoco.MjModel.from_xml_string(self.xml_string)
        self.data = mujoco.MjData(self.model)
        
        # Initialize Renderer (640x480 resolution)
        self.width = 640
        self.height = 480
        self.renderer = mujoco.Renderer(self.model, height=self.height, width=self.width)
        
        self.bridge = CvBridge()
        
        # Start passive 3D viewer to show the full simulation environment
        self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
        
        # Timer for simulation step, render, and publishing (20 Hz)
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.get_logger().info("MuJoCo Integrated Simulator initialized.")

    def timer_callback(self):
        t = self.get_clock().now().nanoseconds * 1e-9
        
        # 1. Keep the Humanoid standing upright at (0.5, -1.0, 1.282) out of the way
        self.data.qpos[0] = 0.5
        self.data.qpos[1] = -1.0
        self.data.qpos[2] = 1.282
        self.data.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
        self.data.qpos[7:28] = 0.0
        
        # 2. Make the AMR move in a circular trajectory of radius 0.6m
        radius = 0.6
        omega = 0.3
        x_amr = radius * np.cos(omega * t)
        y_amr = radius * np.sin(omega * t)
        z_amr = 0.1
        
        # Heading is the tangent of the circular path (yaw)
        heading = omega * t + np.pi / 2.0
        
        # AMR root joint starts at qpos[28:35]
        self.data.qpos[28] = x_amr
        self.data.qpos[29] = y_amr
        self.data.qpos[30] = z_amr
        
        # Quaternion for yaw rotation (heading) around Z-axis
        self.data.qpos[31] = np.cos(heading / 2.0)  # w
        self.data.qpos[32] = 0.0                     # x
        self.data.qpos[33] = 0.0                     # y
        self.data.qpos[34] = np.sin(heading / 2.0)  # z
        
        # Reset velocities to prevent physics integration instabilities (Nan, Inf warning)
        self.data.qvel[:] = 0.0
        
        # Step simulation
        mujoco.mj_step(self.model, self.data)
        
        # Synchronize 3D passive viewer
        if self.viewer.is_running():
            self.viewer.sync()
        
        # 3. Render the scene from the AMR's onboard camera
        self.renderer.update_scene(self.data, camera="onboard_camera")
        rgb_img = self.renderer.render()
        bgr_img = cv2.cvtColor(rgb_img, cv2.COLOR_RGB2BGR)
        
        # Create stamp
        now_msg = self.get_clock().now().to_msg()
        
        # 4. Publish Image
        img_msg = self.bridge.cv2_to_imgmsg(bgr_img, encoding="bgr8")
        img_msg.header.stamp = now_msg
        img_msg.header.frame_id = "camera_link"
        self.img_pub.publish(img_msg)
        
        # 5. Publish CameraInfo (calculated dynamically based on fovy parameter)
        info_msg = CameraInfo()
        info_msg.header.stamp = now_msg
        info_msg.header.frame_id = "camera_link"
        info_msg.width = self.width
        info_msg.height = self.height
        
        fovy_rad = math.radians(self.fovy)
        fy = self.height / (2.0 * math.tan(fovy_rad / 2.0))
        fx = fy  # Square pixels
        cx, cy = 320.0, 240.0
        info_msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info_msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info_msg.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.info_pub.publish(info_msg)
        
        # 6. Publish AMR's global PoseStamped
        pose_msg = PoseStamped()
        pose_msg.header.stamp = now_msg
        pose_msg.header.frame_id = "map"
        pose_msg.pose.position.x = x_amr
        pose_msg.pose.position.y = y_amr
        pose_msg.pose.position.z = z_amr
        pose_msg.pose.orientation.w = self.data.qpos[31]
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = self.data.qpos[34]
        self.pose_pub.publish(pose_msg)
        
        # Save sample image for walkthrough
        static_counter = getattr(self, '_sample_save_counter', 0)
        if static_counter < 10:
            setattr(self, '_sample_save_counter', static_counter + 1)
            if static_counter == 5:
                cv2.imwrite("/home/yoon/.gemini/antigravity/brain/6911cae0-1963-4d05-b904-fabf0acdbae7/amr_onboard_view.png", bgr_img)

        # Apply undistortion if calibration is loaded
        if self.camera_matrix is not None and self.distortion_coeffs is not None:
            bgr_img = cv2.undistort(bgr_img, self.camera_matrix, self.distortion_coeffs)
        
        # Show what the onboard camera sees (undistorted)
        cv2.imshow("AMR Onboard Camera View", bgr_img)
        cv2.waitKey(1)

    def destroy_node(self):
        # Close passive viewer upon node shutdown
        if hasattr(self, 'viewer') and self.viewer.is_running():
            self.viewer.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MujocoCamPublisher()
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
