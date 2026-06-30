#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class SaveDebugImage(Node):
    def __init__(self):
        super().__init__('save_debug_image')
        self.bridge = CvBridge()
        self.count = 0
        self.save_dir = "/home/yoon/yoon_urdf/debug_images"
        os.makedirs(self.save_dir, exist_ok=True)
        self.sub = self.create_subscription(Image, '/camera/image_raw', self.callback, 10)
        self.get_logger().info("SaveDebugImage node started. Saving 10 frames...")

    def callback(self, msg):
        now = self.get_clock().now().nanoseconds * 1e-9
        last_time = getattr(self, '_last_time', 0.0)
        if now - last_time < 2.0:
            return
        setattr(self, '_last_time', now)
        try:
            img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            path = os.path.join(self.save_dir, f"frame_{self.count:03d}.png")
            cv2.imwrite(path, img)
            self.get_logger().info(f"Saved {path}")
            self.count += 1
            if self.count >= 15:
                self.get_logger().info("Finished saving 15 frames. Exiting.")
                rclpy.shutdown()
        except Exception as e:
            self.get_logger().error(f"Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SaveDebugImage()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
