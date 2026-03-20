import cv2
from ultralytics import YOLO

# 1. 내가 학습시킨 YOLO 모델 가중치 파일 불러오기 
# (학습 결과로 나온 'best.pt' 파일의 경로를 적어주세요)
model = YOLO('/home/yoon/yoon_urdf/src/urdf_tuto/drone_detector/drone_detector_01.pt')

# 2. 카메라 열기 
# 아까 확인한 '/dev/video0'에 해당하는 번호인 '0'을 넣습니다.
# 만약 카메라가 여러 대라면 0, 1, 2 등으로 바꿔가며 테스트할 수 있습니다.
cap = cv2.VideoCapture(0)

# 카메라가 정상적으로 열렸는지 확인
if not cap.isOpened():
    print("카메라를 열 수 없습니다. 장치 연결을 확인해주세요.")
    exit()

print("카메라 연동 성공! 종료하려면 영상 창에서 'q'를 누르세요.")

while True:
    # 3. 카메라에서 1프레임씩 이미지 읽어오기
    ret, frame = cap.read()
    
    if not ret:
        print("프레임을 받아올 수 없습니다. 스트림이 끝났거나 오류가 발생했습니다.")
        break

    # 4. 읽어온 이미지(frame)를 YOLO 모델에 넣고 추론(Inference) 실행
    # stream=True를 주면 메모리를 덜 먹어서 실시간 처리에 좋습니다.
    results = model(frame, stream=True)

    # 5. 결과 시각화
    for result in results:
        # bounding box, 라벨 등 결과가 그려진 이미지를 가져옴
        annotated_frame = result.plot()
        
        # 6. 화면에 결과 이미지 띄우기
        cv2.imshow('YOLO Real-time Detection', annotated_frame)

    # 7. 'q' 키를 누르면 무한 루프 종료
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 8. 사용이 끝난 카메라 장치 반환 및 모든 창 닫기
cap.release()
cv2.destroyAllWindows()