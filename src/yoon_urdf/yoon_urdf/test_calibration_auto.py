#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import yaml
import sys
import time

class AutoCalibrateTest(Node):
    def __init__(self):
        super().__init__('auto_calibrate_test')
        self.bridge = CvBridge()
        self.cb_width = 7
        self.cb_height = 5
        self.square_size = 0.1
        
        # Prepare 3d points
        self.objp = np.zeros((self.cb_width * self.cb_height, 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:self.cb_width, 0:self.cb_height].T.reshape(-1, 2) * self.square_size
        
        self.objpoints = []
        self.imgpoints = []
        
        self.captured_count = 0
        self.last_capture_time = 0.0
        self.output_file = '/home/yoon/yoon_urdf/auto_camera_calibration.yaml'
        self.latest_gray_shape = None
        
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        self.get_logger().info("Auto Calibration verification node started. Waiting for checkerboard...")

    def image_callback(self, img_msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            return
            
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        self.latest_gray_shape = gray.shape[::-1]
        
        ret, corners = cv2.findChessboardCorners(gray, (self.cb_width, self.cb_height), None)
        
        if ret:
            # Safe border margin: Reject frames where any corner is within 25 pixels of the image edge
            h, w = gray.shape
            margin = 25.0
            corners_flat = corners.reshape(-1, 2)
            all_inside = True
            for pt in corners_flat:
                cx, cy = pt[0], pt[1]
                if cx < margin or cx > (w - margin) or cy < margin or cy > (h - margin):
                    all_inside = False
                    break
            
            if not all_inside:
                return
                
            now = time.time()
            # Capture at most once every 1.5 seconds to get different perspectives as the robot moves
            if now - self.last_capture_time > 1.5:
                criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
                
                self.objpoints.append(self.objp)
                self.imgpoints.append(corners2)
                self.captured_count += 1
                self.last_capture_time = now
                self.get_logger().info(f"[AUTO] Captured frame #{self.captured_count}")
                
                if self.captured_count >= 8:
                    self.run_calibration()

    def run_calibration(self):
        self.get_logger().info("Calibrating camera matrix with auto-captured frames...")
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
            self.objpoints, self.imgpoints, self.latest_gray_shape, None, None
        )
        if ret:
            calib_data = {
                'image_width': self.latest_gray_shape[0],
                'image_height': self.latest_gray_shape[1],
                'camera_matrix': {
                    'rows': 3,
                    'cols': 3,
                    'data': mtx.flatten().tolist()
                },
                'distortion_coefficients': {
                    'rows': 1,
                    'cols': 5,
                    'data': dist.flatten().tolist()
                }
            }
            with open(self.output_file, 'w') as f:
                yaml.dump(calib_data, f)
            self.get_logger().info(f"Auto-calibration SUCCESS! Saved parameters to: {self.output_file}")
            print("\nAuto Calibration Matrix K:")
            print(mtx)
            print("Auto Distortion D:")
            print(dist.flatten())
            print("\nTEST PASSED successfully!\n")
            sys.exit(0)
        else:
            self.get_logger().error("Auto-calibration solver failed.")
            sys.exit(1)

def main(args=None):
    rclpy.init(args=args)
    node = AutoCalibrateTest()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
