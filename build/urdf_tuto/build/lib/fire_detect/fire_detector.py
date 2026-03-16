import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from inference_sdk import InferenceHTTPClient # Roboflow SDK

class FireDetectorNode(Node):
    def __init__(self):
        super().__init__('fire_detector_node')
        
        # 1. 가제보 로봇 카메라 토픽 구독
        self.subscription = self.create_subscription(
            Image,
            '/my_robot/camera/image_raw', 
            self.image_callback,
            10
        )
        self.bridge = CvBridge()

        # 2. Roboflow API 클라이언트 설정
        self.client = InferenceHTTPClient(
            api_url="https://serverless.roboflow.com",
            api_key="Nda1uCHotUuXPydroEV1" # 사용자 제공 API Key
        )
        self.model_id = "fire-kp9wu-io9vz/1" # 학습시킨 모델 ID
        
        self.get_logger().info('Roboflow 화재 감지 노드가 시작되었습니다.')

    def image_callback(self, msg):
        try:
            # ROS2 이미지 메시지를 OpenCV 포맷으로 변환
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # 화면 밖으로 나가지 않게 리사이징 (640x480)
            display_frame = cv2.resize(cv_image, (640, 480))

            # 3. Roboflow 서버로 프레임 전송 및 화재 감지
            result = self.client.infer(display_frame, model_id=self.model_id)

            # --- 로그 추가 부분 ---
            if result['predictions']:
                self.get_logger().info(f"🔥 불 감지! 예측 수: {len(result['predictions'])}")
            else:
                self.get_logger().info("탐색 중... (불이 보이지 않음)")
            # ----------------------
            
            # 4. 결과 시각화 (바운딩 박스 그리기)
            for prediction in result['predictions']:
                x, y = int(prediction['x']), int(prediction['y'])
                w, h = int(prediction['width']), int(prediction['height'])
                conf = prediction['confidence']
                
                # 좌표 변환 및 박스 그리기
                start_p = (int(x - w/2), int(y - h/2))
                end_p = (int(x + w/2), int(y + h/2))
                cv2.rectangle(display_frame, start_p, end_p, (0, 0, 255), 2)
                cv2.putText(display_frame, f"FIRE {conf:.2f}", (start_p[0], start_p[1]-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

            # 결과 화면 출력
            cv2.imshow("Gazebo Fire Detection (Roboflow API)", display_frame)
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

if __name__ == '__main__':
    main()