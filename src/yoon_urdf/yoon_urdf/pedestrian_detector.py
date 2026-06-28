#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseArray, Pose, PoseStamped
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO
import cv2
import numpy as np
import math

class PedestrianDetector(Node):
    def __init__(self):
        super().__init__('pedestrian_detector')
        
        # Parameters
        self.declare_parameter('detection_mode', 'color')  # 'color' (default for robust testing) or 'yolo'
        self.declare_parameter('model_name', 'yolov8n.pt')
        self.declare_parameter('camera_height', 0.8)       # Camera height on AMR in meters
        self.declare_parameter('camera_offset_x', 0.25)    # Camera forward offset from AMR center in meters
        self.declare_parameter('focal_length_x', 500.0)     # Default focal length (pixel)
        self.declare_parameter('focal_length_y', 500.0)
        self.declare_parameter('center_x', 320.0)           # Default principal point (pixel)
        self.declare_parameter('center_y', 240.0)
        
        # Height standards for depth estimation
        self.declare_parameter('pedestrian_height_m', 1.7) # Typical pedestrian height in meters
        
        self.detection_mode = self.get_parameter('detection_mode').value
        model_name = self.get_parameter('model_name').value
        self.camera_height = self.get_parameter('camera_height').value
        self.camera_offset_x = self.get_parameter('camera_offset_x').value
        self.fx = self.get_parameter('focal_length_x').value
        self.fy = self.get_parameter('focal_length_y').value
        self.cx = self.get_parameter('center_x').value
        self.cy = self.get_parameter('center_y').value
        self.ped_height = self.get_parameter('pedestrian_height_m').value
        
        self.get_logger().info(f"Detector running in mode: '{self.detection_mode}'")
        
        # Load YOLO model only if mode is 'yolo'
        if self.detection_mode == 'yolo':
            self.get_logger().info(f"Loading YOLO Model: {model_name}...")
            self.model = YOLO(model_name)
            self.get_logger().info("YOLO Model loaded successfully.")
        
        self.bridge = CvBridge()
        
        # Robot global pose tracking
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.robot_pose_received = False
        
        # Subscribers
        self.img_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.info_sub = self.create_subscription(
            CameraInfo, '/camera/camera_info', self.camera_info_callback, 10)
        self.pose_sub = self.create_subscription(
            PoseStamped, '/robot_global_pose', self.robot_pose_callback, 10)
            
        # Publishers
        self.pose_pub = self.create_publisher(PoseArray, '/pedestrian_states', 10)
        
        self.get_logger().info("Detector Node initialized.")

    def robot_pose_callback(self, msg: PoseStamped):
        self.robot_x = msg.pose.position.x
        self.robot_y = msg.pose.position.y
        # Convert quaternion to yaw angle
        q = msg.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)
        self.robot_pose_received = True

    def camera_info_callback(self, msg: CameraInfo):
        # Dynamically load camera calibration parameters if available
        if msg.k[0] > 0.0:
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]

    def image_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error: {e}")
            return
            
        pose_array_msg = PoseArray()
        pose_array_msg.header.stamp = msg.header.stamp
        pose_array_msg.header.frame_id = 'map'  # Map frame coordinates
        
        if self.detection_mode == 'color':
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            lower_red1 = np.array([0, 100, 100])
            upper_red1 = np.array([10, 255, 255])
            lower_red2 = np.array([170, 100, 100])
            upper_red2 = np.array([180, 255, 255])
            
            mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
            mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
            mask = cv2.bitwise_or(mask1, mask2)
            
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if contours:
                valid_contours = [c for c in contours if cv2.contourArea(c) > 30]
                if valid_contours:
                    largest_contour = max(valid_contours, key=cv2.contourArea)
                    x, y, w, h = cv2.boundingRect(largest_contour)
                    h_pixel = float(h)
                    u = x + w / 2.0
                    v = y + h / 2.0
                    
                    if h_pixel > 0.0:
                        # Depth estimation: target real height = 1.7m
                        distance = (self.ped_height * self.fy) / h_pixel
                        z_cam = distance
                        x_cam = ((u - self.cx) * z_cam) / self.fx
                        y_cam = ((v - self.cy) * z_cam) / self.fy
                        
                        # Robot local coordinates (taking camera x offset into account)
                        x_robot = z_cam + self.camera_offset_x
                        y_robot = -x_cam
                        z_robot = -y_cam + self.camera_height
                        
                        # Transform local coordinates to global map coordinates using AMR global pose
                        if self.robot_pose_received:
                            x_global = self.robot_x + x_robot * math.cos(self.robot_yaw) - y_robot * math.sin(self.robot_yaw)
                            y_global = self.robot_y + x_robot * math.sin(self.robot_yaw) + y_robot * math.cos(self.robot_yaw)
                        else:
                            x_global = x_robot
                            y_global = y_robot
                            
                        # PRINT DEBUG INFO
                        self.get_logger().info(
                            f"\n--- DEBUG ---\n"
                            f"h_pixel: {h_pixel:.1f}, distance: {distance:.2f}m\n"
                            f"Robot global: ({self.robot_x:.2f}, {self.robot_y:.2f}), yaw: {self.robot_yaw:.2f} rad\n"
                            f"Local coordinates: x={x_robot:.2f}, y={y_robot:.2f}\n"
                            f"Global coordinates: X={x_global:.2f}, Y={y_global:.2f}\n"
                            f"-------------",
                            throttle_duration_sec=1.0
                        )
                            
                        pose = Pose()
                        pose.position.x = x_global
                        pose.position.y = y_global
                        pose.position.z = max(0.0, z_robot)
                        pose.orientation.w = 1.0
                        pose_array_msg.poses.append(pose)
                        
                        cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0, 0, 255), 2)
                        cv2.putText(cv_image, f"Target: {distance:.2f}m", (x, y - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            
            cv2.imshow("Detector - AMR Color Tracking Stream", cv_image)
            cv2.waitKey(1)
            
        elif self.detection_mode == 'yolo':
            results = self.model.predict(cv_image, classes=[0], verbose=False)
            boxes = results[0].boxes
            for box in boxes:
                coords = box.xyxy[0].cpu().numpy()
                x1, y1, x2, y2 = coords
                h_pixel = y2 - y1
                u = (x1 + x2) / 2.0
                v = (y1 + y2) / 2.0
                
                if h_pixel <= 0.0:
                    continue
                distance = (self.ped_height * self.fy) / h_pixel
                
                z_cam = distance
                x_cam = ((u - self.cx) * z_cam) / self.fx
                y_cam = ((v - self.cy) * z_cam) / self.fy
                
                # Robot local coordinates (taking camera x offset into account)
                x_robot = z_cam + self.camera_offset_x
                y_robot = -x_cam
                z_robot = -y_cam + self.camera_height
                
                # Transform local coordinates to global map coordinates using AMR global pose
                if self.robot_pose_received:
                    x_global = self.robot_x + x_robot * math.cos(self.robot_yaw) - y_robot * math.sin(self.robot_yaw)
                    y_global = self.robot_y + x_robot * math.sin(self.robot_yaw) + y_robot * math.cos(self.robot_yaw)
                else:
                    x_global = x_robot
                    y_global = y_robot
                
                pose = Pose()
                pose.position.x = x_global
                pose.position.y = y_global
                pose.position.z = z_robot
                pose.orientation.w = 1.0
                pose_array_msg.poses.append(pose)
                
            annotated_frame = results[0].plot()
            cv2.imshow("Detector - YOLO Pedestrian Detections", annotated_frame)
            cv2.waitKey(1)
            
        # Publish estimated poses (in map frame)
        self.pose_pub.publish(pose_array_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PedestrianDetector()
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
