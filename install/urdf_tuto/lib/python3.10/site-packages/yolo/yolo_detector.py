import os
os.environ["QT_QPA_PLATFORM"] = "xcb"

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2

class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')

        self.subscription = self.create_subscription(
            Image,
            '/my_robot/camera/image_raw',
            self.image_callback,
            10
        )

        self.pub = self.create_publisher(Image, '/yolo/annotated', 10)

        self.bridge = CvBridge()
        self.model = YOLO('yolov8s.pt')

        self.get_logger().info('YOLO Detector Node가 시작되었습니다.')

    def image_callback(self, msg):
        try:
            self.get_logger().info('callback entered')

            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.get_logger().info(f'frame shape: {cv_image.shape}')

            results = self.model(cv_image, conf=0.5, verbose=False)

            if results and len(results) > 0:
                annotated_frame = results[0].plot()
            else:
                annotated_frame = cv_image

            out_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')
            out_msg.header = msg.header

            self.pub.publish(out_msg)
            self.get_logger().info('published /yolo/annotated')

        except Exception as e:
            self.get_logger().error(f'이미지 처리 중 에러 발생: {e}')

def main(args=None):
    rclpy.init(args=args)
    yolo_detector = YoloDetector()
    try:
        rclpy.spin(yolo_detector)
    except KeyboardInterrupt:
        pass
    finally:
        yolo_detector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()