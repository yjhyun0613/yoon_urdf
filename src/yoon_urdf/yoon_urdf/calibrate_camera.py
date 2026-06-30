#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import yaml
import sys
import os

class CalibrateCameraNode(Node):
    def __init__(self):
        super().__init__('calibrate_camera')
        
        # Parameters for chessboard grid dimensions (inner corners)
        self.declare_parameter('chessboard_width', 7)  # standard for 8 columns -> 7 inner corners
        self.declare_parameter('chessboard_height', 5) # standard for 6 rows -> 5 inner corners
        self.declare_parameter('square_size', 0.1)    # grid square side length in meters
        self.declare_parameter('output_file', '/home/yoon/yoon_urdf/camera_calibration.yaml')
        
        self.cb_width = self.get_parameter('chessboard_width').value
        self.cb_height = self.get_parameter('chessboard_height').value
        self.square_size = self.get_parameter('square_size').value
        self.output_file = self.get_parameter('output_file').value
        
        self.bridge = CvBridge()
        
        # Prepare 3D object points based on chessboard grid size
        # Format: (0,0,0), (1,0,0), (2,0,0) ...., (cb_width-1, cb_height-1, 0)
        self.objp = np.zeros((self.cb_width * self.cb_height, 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:self.cb_width, 0:self.cb_height].T.reshape(-1, 2) * self.square_size
        
        # Arrays to store object points and image points from all captured images
        self.objpoints = [] # 3d point in real world space
        self.imgpoints = [] # 2d points in image plane
        
        self.captured_count = 0
        self.latest_corners = None
        self.latest_gray_shape = None
        self.latest_cv_image = None
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        
        # Subscription
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        self.window_name = "ROS2 Interactive Camera Calibration"
        self.get_logger().info(f"Calibration node initialized. Target grid: {self.cb_width}x{self.cb_height} inner corners.")
        self.get_logger().info("=========================================")
        self.get_logger().info(" Keyboard Controls in Image Window:")
        self.get_logger().info("  [c] : Capture current frame (if grid is green/detected)")
        self.get_logger().info("  [s] : Solve camera calibration parameters and save to YAML")
        self.get_logger().info("  [q] : Quit without saving")
        self.get_logger().info("=========================================")

    def image_callback(self, img_msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Decode Error: {e}")
            return
            
        h, w = cv_image.shape[:2]
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        self.latest_gray_shape = gray.shape[::-1]
        self.latest_cv_image = cv_image.copy()
        
        # Find chessboard corners
        ret, corners = cv2.findChessboardCorners(
            gray, 
            (self.cb_width, self.cb_height), 
            cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_FAST_CHECK
        )
        
        display_img = cv_image.copy()
        
        if ret:
            # Refine corner locations to sub-pixel accuracy
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), self.criteria)
            self.latest_corners = corners2
            
            # Draw Chessboard Corners (Green indicates ready to capture)
            cv2.drawChessboardCorners(display_img, (self.cb_width, self.cb_height), corners2, ret)
            cv2.putText(display_img, "Chessboard DETECTED! Press 'c' to capture.", (15, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        else:
            self.latest_corners = None
            cv2.putText(display_img, "Searching for Checkerboard...", (15, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                        
        # Display capture status
        cv2.putText(display_img, f"Captured Frames: {self.captured_count}", (15, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        cv2.putText(display_img, "[c]: Capture  [s]: Solve & Save  [q]: Quit", (15, h - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                    
        cv2.imshow(self.window_name, display_img)
        key = cv2.waitKey(1) & 0xFF
        
        if key == ord('c'):
            self.capture_frame()
        elif key == ord('s'):
            self.solve_calibration()
        elif key == ord('q'):
            self.get_logger().info("Exiting calibration node...")
            cv2.destroyAllWindows()
            sys.exit(0)

    def capture_frame(self):
        if self.latest_corners is not None:
            self.objpoints.append(self.objp)
            self.imgpoints.append(self.latest_corners)
            self.captured_count += 1
            self.get_logger().info(f"Captured frame #{self.captured_count} successfully.")
            
            # Show a green flash feedback on screen
            flash = np.zeros_like(self.latest_cv_image)
            flash[:, :] = [0, 255, 0]
            feedback = cv2.addWeighted(self.latest_cv_image, 0.7, flash, 0.3, 0)
            cv2.imshow(self.window_name, feedback)
            cv2.waitKey(150)
        else:
            self.get_logger().warn("Chessboard NOT detected! Cannot capture.")

    def solve_calibration(self):
        if self.captured_count < 5:
            self.get_logger().error(f"Need at least 5 frames to calibrate. Currently captured: {self.captured_count}")
            return
            
        self.get_logger().info("Computing camera calibration parameters (solving DLT & Levenberg-Marquardt)...")
        
        try:
            ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
                self.objpoints, 
                self.imgpoints, 
                self.latest_gray_shape, 
                None, 
                None
            )
            
            if ret:
                self.get_logger().info("Calibration solved successfully!")
                
                # Format parameters for YAML
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
                
                # Write to output file
                os.makedirs(os.path.dirname(self.output_file), exist_ok=True)
                with open(self.output_file, 'w') as f:
                    yaml.dump(calib_data, f, default_flow_style=False)
                    
                self.get_logger().info(f"Calibration files successfully saved to: {self.output_file}")
                
                print("\n" + "="*50)
                print(" CAMERA CALIBRATION RESULTS ")
                print("="*50)
                print("Camera Matrix K:")
                print(mtx)
                print("-"*50)
                print("Distortion Coefficients D:")
                print(dist.flatten())
                print("="*50 + "\n")
                
                # Flash white on screen to indicate success
                flash = np.ones_like(self.latest_cv_image) * 255
                feedback = cv2.addWeighted(self.latest_cv_image, 0.5, flash, 0.5, 0)
                cv2.imshow(self.window_name, feedback)
                cv2.waitKey(300)
                
            else:
                self.get_logger().error("Calibration Solver failed.")
        except Exception as e:
            self.get_logger().error(f"Error during calibration calculation: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CalibrateCameraNode()
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
