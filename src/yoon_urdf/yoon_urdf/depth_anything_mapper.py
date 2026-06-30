#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge, CvBridgeError
import cv2
import torch
import numpy as np
import struct
import math
import time
from transformers import AutoImageProcessor, AutoModelForDepthEstimation
import yaml
import os

class DepthAnythingMapper(Node):
    def __init__(self):
        super().__init__('depth_anything_mapper')
        
        # Parameters
        self.declare_parameter('camera_offset_x', 0.25)
        self.declare_parameter('camera_height', 0.8)
        self.declare_parameter('downsample_step', 4)  # Lower is more precise but heavier
        self.declare_parameter('min_depth', 0.5)      # Metric mapping range minimum (m)
        self.declare_parameter('max_depth', 8.0)      # Metric mapping range maximum (m)
        self.declare_parameter('model_id', 'depth-anything/Depth-Anything-V2-Small-hf')
        self.declare_parameter('roi_margin', 0.12)     # Crop ratio for boundary distortion filtering
        
        self.camera_offset_x = self.get_parameter('camera_offset_x').value
        self.camera_height = self.get_parameter('camera_height').value
        self.downsample_step = self.get_parameter('downsample_step').value
        self.min_depth = self.get_parameter('min_depth').value
        self.max_depth = self.get_parameter('max_depth').value
        self.roi_margin = self.get_parameter('roi_margin').value
        model_id = self.get_parameter('model_id').value
        
        self.bridge = CvBridge()
        
        # Camera Calibration state (populated dynamically by CameraInfo topic or calibration file)
        self.camera_matrix = np.eye(3)
        self.distortion_coeffs = np.zeros(5)
        self.fx = 320.0
        self.fy = 240.0
        self.cx = 320.0
        self.cy = 240.0
        self.camera_info_received = False
        
        # Load calibration parameters from auto_camera_calibration.yaml if exists
        yaml_path = "/home/yoon/yoon_urdf/auto_camera_calibration.yaml"
        if os.path.exists(yaml_path):
            try:
                with open(yaml_path, 'r') as f:
                    calib_data = yaml.safe_load(f)
                self.camera_matrix = np.array(calib_data['camera_matrix']['data']).reshape((3, 3))
                self.distortion_coeffs = np.array(calib_data['distortion_coefficients']['data'])
                self.fx = self.camera_matrix[0, 0]
                self.fy = self.camera_matrix[1, 1]
                self.cx = self.camera_matrix[0, 2]
                self.cy = self.camera_matrix[1, 2]
                self.camera_info_received = True
                self.get_logger().info(f"Successfully loaded calibration parameters from {yaml_path}")
            except Exception as e:
                self.get_logger().error(f"Failed to load calibration from {yaml_path}: {e}")
        
        # Pose tracking state
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.robot_pose_received = False
        
        # Setup Depth Anything V2 model
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.get_logger().info(f"Loading Depth Anything model: {model_id} on device: {self.device}...")
        
        try:
            self.image_processor = AutoImageProcessor.from_pretrained(model_id)
            self.model = AutoModelForDepthEstimation.from_pretrained(model_id).to(self.device)
            self.get_logger().info("Model loaded successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to load Hugging Face model: {e}")
            raise e
            
        # Subscribers
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/robot_global_pose',
            self.pose_callback,
            10
        )
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        self.info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.camera_info_callback,
            10
        )
        
        # Publisher
        self.pointcloud_pub = self.create_publisher(PointCloud2, '/semantic_pointcloud', 10)
        
        self.get_logger().info("Depth Anything Mapper Node initialized.")

    def camera_info_callback(self, msg: CameraInfo):
        # Dynamically extract intrinsics and distortion coefficients
        self.camera_matrix = np.array(msg.k).reshape((3, 3))
        self.distortion_coeffs = np.array(msg.d)
        
        # Extract focal lengths and center points
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]
        
        self.camera_info_received = True

    def pose_callback(self, msg: PoseStamped):
        self.robot_x = msg.pose.position.x
        self.robot_y = msg.pose.position.y
        
        # Euler extraction from Quaternion (Yaw only)
        q = msg.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)
        self.robot_pose_received = True

    def image_callback(self, img_msg: Image):
        if not self.robot_pose_received:
            self.get_logger().warn("Waiting for robot pose...", throttle_duration_sec=5.0)
            return
            
        # 1. Convert ROS Image to OpenCV BGR
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Decode Error: {e}")
            return
            
        h, w = cv_image.shape[:2]
        
        # Apply lens undistortion pre-processing if camera calibration info is available
        if self.camera_info_received:
            cv_image = cv2.undistort(cv_image, self.camera_matrix, self.distortion_coeffs)
        
        # Update dynamic parameters
        self.min_depth = self.get_parameter('min_depth').value
        self.max_depth = self.get_parameter('max_depth').value
        self.downsample_step = self.get_parameter('downsample_step').value
        self.roi_margin = self.get_parameter('roi_margin').value
        
        # 2. Run Depth Anything V2 Inference
        t_start = time.time()
        
        inputs = self.image_processor(images=cv_image, return_tensors="pt").to(self.device)
        with torch.no_grad():
            outputs = self.model(**inputs)
            predicted_depth = outputs.predicted_depth
            
        # 3. Resize predicted depth to match original resolution
        prediction = torch.nn.functional.interpolate(
            predicted_depth.unsqueeze(1),
            size=(h, w),
            mode="bicubic",
            align_corners=False,
        ).squeeze()
        
        depth_np = prediction.cpu().numpy()
        t_end = time.time()
        self.get_logger().info(f"AI Depth Inference time: {(t_end - t_start)*1000:.1f}ms", throttle_duration_sec=3.0)
        
        # 4. Scale normalization and Metric mapping
        # Higher prediction value means closer. Let's normalize it to [0, 1]
        d_min = depth_np.min()
        d_max = depth_np.max()
        if d_max - d_min > 1e-5:
            d_norm = (depth_np - d_min) / (d_max - d_min)
        else:
            d_norm = np.zeros_like(depth_np)
            
        # Inverse depth (disparity) mapping: 1.0 (closest) -> min_depth, 0.0 (furthest) -> max_depth
        depth_metric = 1.0 / ( (1.0 / self.max_depth) + d_norm * ( (1.0 / self.min_depth) - (1.0 / self.max_depth) ) )
        
        # 5. Project to 3D Points
        semantic_points = []
        
        U_bg, V_bg = np.meshgrid(np.arange(0, w, self.downsample_step), np.arange(0, h, self.downsample_step))
        Z_metric = depth_metric[V_bg, U_bg]
        
        # Valid range filter and ROI margin filter to remove border distortion
        valid_mask = (Z_metric >= self.min_depth) & (Z_metric <= self.max_depth)
        if self.roi_margin > 0.0:
            margin_x = int(w * self.roi_margin)
            margin_y = int(h * self.roi_margin)
            valid_mask &= (U_bg >= margin_x) & (U_bg <= w - margin_x)
            valid_mask &= (V_bg >= margin_y) & (V_bg <= h - margin_y)
            
        U_valid = U_bg[valid_mask]
        V_valid = V_bg[valid_mask]
        Z_valid = Z_metric[valid_mask]
        
        # Camera local coordinates
        X_cam = (U_valid - self.cx) * Z_valid / self.fx
        Y_cam = (V_valid - self.cy) * Z_valid / self.fy
        
        # Robot local coordinates (Optical convention conversion)
        X_robot = Z_valid + self.camera_offset_x
        Y_robot = -X_cam
        Z_robot = -Y_cam + self.camera_height
        
        # Global map coordinates
        X_global = X_robot * math.cos(self.robot_yaw) - Y_robot * math.sin(self.robot_yaw) + self.robot_x
        Y_global = X_robot * math.sin(self.robot_yaw) + Y_robot * math.cos(self.robot_yaw) + self.robot_y
        Z_global = Z_robot
        
        if len(X_global) > 0:
            # Color extraction from BGR image
            BGR = cv_image[V_valid, U_valid]
            B = BGR[:, 0].astype(np.uint32)
            G = BGR[:, 1].astype(np.uint32)
            R = BGR[:, 2].astype(np.uint32)
            RGB = (R << 16) | (G << 8) | B
            
            semantic_points = list(zip(X_global, Y_global, Z_global, RGB))
            
        # 6. Build and Publish PointCloud2
        pc_msg = self.create_pointcloud_msg(img_msg.header, semantic_points)
        self.pointcloud_pub.publish(pc_msg)

    def create_pointcloud_msg(self, header, points):
        msg = PointCloud2()
        msg.header = header
        msg.header.frame_id = 'map'
        msg.height = 1
        msg.width = len(points)
        
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        
        buffer = []
        for p in points:
            buffer.append(struct.pack('fffI', p[0], p[1], p[2], p[3]))
        msg.data = b''.join(buffer)
        
        return msg

def main(args=None):
    rclpy.init(args=args)
    node = DepthAnythingMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
