import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO  # 로컬 YOLO 라이브러리

class FireDetectorNode(Node):
    def __init__(self):
        super().__init__('drone_detector_node')
        
        self.subscription = self.create_subscription(
            Image,
            '/my_robot/camera/image_raw', 
            self.image_callback,
            10
        )
        self.bridge = CvBridge()

        # 1. 내 컴퓨터의 GPU(RTX 3070)를 사용하여 모델 로드
        # 경로를 'best.pt'로 설정 (파일이 실행 위치에 있어야 함)
        self.model = YOLO('/home/yoon/yoon_urdf/src/urdf_tuto/drone_detector/drone_detector_01.pt').to('cuda') 
        
        self.get_logger().info('드론 감지를 시작합니다!')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # 2. 로컬 GPU에서 즉시 추론 (인터넷 안 거침, 매우 빠름)
            results = self.model.predict(cv_image, conf=0.5, verbose=False)

            # 3. 결과 시각화 (YOLO 라이브러리의 자동 그리기 기능 활용)
            annotated_frame = results[0].plot()

            # 감지 로그 출력
            if len(results[0].boxes) > 0:
                self.get_logger().info(f"드론 불 감지! ({len(results[0].boxes)}개)")

            # 결과 화면 출력
            cv2.imshow("Real-time Local Detection", annotated_frame)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f'이미지 처리 중 에러 발생: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = FireDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()