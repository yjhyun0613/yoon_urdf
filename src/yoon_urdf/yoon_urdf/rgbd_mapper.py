#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import math
import struct
import message_filters
from message_filters import Subscriber, ApproximateTimeSynchronizer
from ultralytics import YOLO
import yaml
import os

class RgbdMapper(Node):
    def __init__(self):
        super().__init__('rgbd_mapper')
        
        # Parameters
        self.declare_parameter('detection_mode', 'color')  # 'color' or 'yolo'
        self.declare_parameter('model_name', 'yolov8n.pt')
        self.declare_parameter('camera_offset_x', 0.25)
        self.declare_parameter('camera_height', 0.8)
        self.declare_parameter('downsample_step', 3)       # Background downsampling step (high precision)
        self.declare_parameter('pedestrian_step', 1)       # Pedestrian box downsampling step (full resolution)
        
        self.detection_mode = self.get_parameter('detection_mode').value
        model_name = self.get_parameter('model_name').value
        self.camera_offset_x = self.get_parameter('camera_offset_x').value
        self.camera_height = self.get_parameter('camera_height').value
        self.downsample_step = self.get_parameter('downsample_step').value
        self.pedestrian_step = self.get_parameter('pedestrian_step').value
        
        self.get_logger().info(f"RGB-D Mapper running in mode: '{self.detection_mode}'")
        
        # Load YOLO model only if mode is 'yolo'
        if self.detection_mode == 'yolo':
            self.get_logger().info(f"Loading YOLO Model: {model_name}...")
            self.model = YOLO(model_name)
            self.get_logger().info("YOLO Model loaded successfully.")
            
        self.bridge = CvBridge()
        
        # Camera Intrinsics (Dynamic from CameraInfo or calibration file)
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.camera_matrix = None
        self.distortion_coeffs = None
        
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
                self.get_logger().info(f"Successfully loaded calibration parameters from {yaml_path}")
            except Exception as e:
                self.get_logger().error(f"Failed to load calibration from {yaml_path}: {e}")
        
        # Robot global pose
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.robot_pose_received = False
        
        # message_filters Subscribers
        self.img_sub = Subscriber(self, Image, '/camera/image_raw')
        self.depth_sub = Subscriber(self, Image, '/camera/depth/image_raw')
        self.pose_sub = Subscriber(self, PoseStamped, '/robot_global_pose')
        
        # ApproximateTimeSynchronizer (slop=0.05s)
        self.ts = ApproximateTimeSynchronizer(
            [self.img_sub, self.depth_sub, self.pose_sub],
            queue_size=10,
            slop=0.05
        )
        self.ts.registerCallback(self.sync_callback)
        
        # Standard subscription for CameraInfo (static intrinsics)
        self.info_sub = self.create_subscription(
            CameraInfo, '/camera/camera_info', self.camera_info_callback, 10)
            
        # Publishers
        self.pointcloud_pub = self.create_publisher(PointCloud2, '/semantic_pointcloud', 10)
        
        self.get_logger().info("RGB-D Mapper Node initialized.")
 
    def camera_info_callback(self, msg: CameraInfo):
        if msg.k[0] > 0.0:
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]
            self.camera_matrix = np.array(msg.k).reshape((3, 3))
            self.distortion_coeffs = np.array(msg.d)

    def sync_callback(self, img_msg, depth_msg, pose_msg):
        # 1. Ensure camera calibration is received
        if self.fx is None:
            self.get_logger().warn("Waiting for camera_info...", throttle_duration_sec=3.0)
            return
            
        # 2. Extract robot pose
        self.robot_x = pose_msg.pose.position.x
        self.robot_y = pose_msg.pose.position.y
        q = pose_msg.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)
        self.robot_pose_received = True
        
        # 3. Decode images
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
            # Depth image is 32FC1 (float32 array containing distance in meters)
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Decode Error: {e}")
            return
            
        h, w = depth_image.shape[:2]
        
        # Apply lens undistortion pre-processing if camera calibration info is available
        if self.camera_matrix is not None:
            cv_image = cv2.undistort(cv_image, self.camera_matrix, self.distortion_coeffs)
            depth_image = cv2.undistort(depth_image, self.camera_matrix, self.distortion_coeffs)
        
        # 4. Perform Pedestrian Detection to get Bounding Boxes
        ped_boxes = []
        if self.detection_mode == 'yolo':
            results = self.model.predict(cv_image, classes=[0], verbose=False)
            for box in results[0].boxes:
                coords = box.xyxy[0].cpu().numpy()
                ped_boxes.append(coords)  # [x1, y1, x2, y2]
        elif self.detection_mode == 'color':
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            lower_red1 = np.array([0, 100, 100])
            upper_red1 = np.array([10, 255, 255])
            lower_red2 = np.array([170, 100, 100])
            upper_red2 = np.array([180, 255, 255])
            mask = cv2.bitwise_or(cv2.inRange(hsv, lower_red1, upper_red1), cv2.inRange(hsv, lower_red2, upper_red2))
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for c in contours:
                if cv2.contourArea(c) > 30:
                    bx, by, bw, bh = cv2.boundingRect(c)
                    ped_boxes.append([bx, by, bx+bw, by+bh])
                    
        # Helper to check if pixel falls inside any pedestrian bounding box
        def in_ped_box(u, v):
            for box in ped_boxes:
                if box[0] <= u <= box[2] and box[1] <= v <= box[3]:
                    return True
            return False

        # 5. Project Depth to 3D Points
        semantic_points = []  # Elements: (X_global, Y_global, Z_global, RGB_packed)
        
        # A. Background Point Cloud (downsampled by downsample_step)
        U_bg, V_bg = np.meshgrid(np.arange(0, w, self.downsample_step), np.arange(0, h, self.downsample_step))
        depth_bg = depth_image[V_bg, U_bg]
        
        # Valid depth range (0.1m to 10m)
        valid_bg = (depth_bg > 0.1) & (depth_bg < 10.0)
        
        U_bg_valid = U_bg[valid_bg]
        V_bg_valid = V_bg[valid_bg]
        Z_bg_valid = depth_bg[valid_bg]
        
        # Camera local coordinates
        X_cam_bg = (U_bg_valid - self.cx) * Z_bg_valid / self.fx
        Y_cam_bg = (V_bg_valid - self.cy) * Z_bg_valid / self.fy
        
        # Robot local coordinates
        X_robot_bg = Z_bg_valid + self.camera_offset_x
        Y_robot_bg = -X_cam_bg
        Z_robot_bg = -Y_cam_bg + self.camera_height
        
        # Global map coordinates
        X_global_bg = X_robot_bg * math.cos(self.robot_yaw) - Y_robot_bg * math.sin(self.robot_yaw) + self.robot_x
        Y_global_bg = X_robot_bg * math.sin(self.robot_yaw) + Y_robot_bg * math.cos(self.robot_yaw) + self.robot_y
        Z_global_bg = Z_robot_bg
        
        if len(X_global_bg) > 0:
            # Vectorized color fetching from cv_image (BGR)
            BGR_bg = cv_image[V_bg_valid, U_bg_valid]
            B_bg = BGR_bg[:, 0].astype(np.uint32)
            G_bg = BGR_bg[:, 1].astype(np.uint32)
            R_bg = BGR_bg[:, 2].astype(np.uint32)
            RGB_bg = (R_bg << 16) | (G_bg << 8) | B_bg
            
            semantic_points.extend(list(zip(X_global_bg, Y_global_bg, Z_global_bg, RGB_bg)))
            
        # B. Dense Pedestrian Point Cloud (processed inside bboxes with pedestrian_step)
        for box in ped_boxes:
            x1, y1, x2, y2 = map(int, box)
            x1, y1 = max(0, x1), max(0, y1)
            x2, y2 = min(w - 1, x2), min(h - 1, y2)
            
            if x2 <= x1 or y2 <= y1:
                continue
                
            U_ped, V_ped = np.meshgrid(np.arange(x1, x2 + 1, self.pedestrian_step), np.arange(y1, y2 + 1, self.pedestrian_step))
            depth_ped = depth_image[V_ped, U_ped]
            
            valid_ped = (depth_ped > 0.1) & (depth_ped < 10.0)
            
            U_ped_valid = U_ped[valid_ped]
            V_ped_valid = V_ped[valid_ped]
            Z_ped_valid = depth_ped[valid_ped]
            
            if len(Z_ped_valid) == 0:
                continue
                
            X_cam_ped = (U_ped_valid - self.cx) * Z_ped_valid / self.fx
            Y_cam_ped = (V_ped_valid - self.cy) * Z_ped_valid / self.fy
            
            X_robot_ped = Z_ped_valid + self.camera_offset_x
            Y_robot_ped = -X_cam_ped
            Z_robot_ped = -Y_cam_ped + self.camera_height
            
            X_global_ped = X_robot_ped * math.cos(self.robot_yaw) - Y_robot_ped * math.sin(self.robot_yaw) + self.robot_x
            Y_global_ped = X_robot_ped * math.sin(self.robot_yaw) + Y_robot_ped * math.cos(self.robot_yaw) + self.robot_y
            Z_global_ped = Z_robot_ped
            
            # Fetch actual colors for pedestrian points
            BGR_ped = cv_image[V_ped_valid, U_ped_valid]
            B_ped = BGR_ped[:, 0].astype(np.uint32)
            G_ped = BGR_ped[:, 1].astype(np.uint32)
            R_ped = BGR_ped[:, 2].astype(np.uint32)
            RGB_ped = (R_ped << 16) | (G_ped << 8) | B_ped
            
            semantic_points.extend(list(zip(X_global_ped, Y_global_ped, Z_global_ped, RGB_ped)))
                
        # 6. Create PointCloud2 ROS2 Message
        pc_msg = self.create_pointcloud_msg(img_msg.header, semantic_points)
        self.pointcloud_pub.publish(pc_msg)
        
        self.get_logger().info(
            f"RGB-D Point Cloud published. Points count: {len(semantic_points)} (Pedestrians: {len(ped_boxes)})",
            throttle_duration_sec=2.0
        )

    def create_pointcloud_msg(self, header, points):
        msg = PointCloud2()
        msg.header = header
        msg.header.frame_id = 'map'
        msg.height = 1
        msg.width = len(points)
        
        # Define fields: x, y, z, rgb
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
        
        # Pack data using struct
        buffer = []
        for p in points:
            # float, float, float, uint32
            buffer.append(struct.pack('fffI', p[0], p[1], p[2], p[3]))
        msg.data = b''.join(buffer)
        return msg

def main(args=None):
    rclpy.init(args=args)
    node = RgbdMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
