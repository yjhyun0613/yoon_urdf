import cv2
from inference_sdk import InferenceHTTPClient

# 1. 클라이언트 설정
CLIENT = InferenceHTTPClient(
    api_url="https://serverless.roboflow.com",
    api_key="Nda1uCHotUuXPydroEV1"
)

# 2. 영상 경로 설정 (입력하신 경로 그대로 사용)
video_path = '/home/yoon/yoon_urdf/src/urdf_tuto/fire_detect/2026_03_01 14_23_1.mp4'
cap = cv2.VideoCapture(video_path)

# 영상이 제대로 열렸는지 확인
if not cap.isOpened():
    print("영상을 불러올 수 없습니다. 경로를 다시 확인해 주세요.")
    exit()

while cap.isOpened():
    success, frame = cap.read()
    if not success:
        break
    
    frame = cv2.resize(frame, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)
    # 3. 중요: 현재 프레임(frame)을 서버로 보내서 추론
    # model_id는 알려주신 것을 사용합니다.
    result = CLIENT.infer(frame, model_id="fire-kp9wu-io9vz/1")
    
    # 4. 결과 시각화 (화면에 사각형 그리기)
    for prediction in result['predictions']:
        x, y = int(prediction['x']), int(prediction['y'])
        w, h = int(prediction['width']), int(prediction['height'])
        
        # 좌표 계산 (중심점 기반을 좌상단/우하단으로 변환)
        start_point = (int(x - w/2), int(y - h/2))
        end_point = (int(x + w/2), int(y + h/2))
        
        # OpenCV로 프레임에 박스 그리기
        cv2.rectangle(frame, start_point, end_point, (0, 0, 255), 2)
        cv2.putText(frame, f"Fire: {prediction['confidence']:.2f}", 
                    (start_point[0], start_point[1]-10), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

    # 5. 결과 화면 표시
    cv2.imshow("Fire Detection Test", frame)

    # 'q'를 누르면 종료
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()