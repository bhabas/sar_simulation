from ultralytics import YOLO
import cv2

model = YOLO("yolov8n-seg.pt")  # load an official model

video_path = "input_video.mp4"

target_size = (640, 360)

cap = cv2.VideoCapture(video_path)

## LOOP THROUGH VIDEO FRAMES    
while cap.isOpened():

    # Read a frame for the video
    success, frame = cap.read()
    frame = cv2.resize(frame, target_size, interpolation=cv2.INTER_LINEAR)

    if success:
        # Run YOLOv7 inference on the frame
        results = model(frame)

        # Visualize the results ont he frame
        annotated_frame = results[0].plot()

        # Display the annotated frame
        cv2.imshow("YOLOv8 Inference", annotated_frame)

        # Break the loop when 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    else:
        break


cap.release()
cv2.destroyAllWindows()
