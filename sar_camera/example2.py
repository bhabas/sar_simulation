from ultralytics import YOLO
import cv2

model = YOLO("models/yolov8n-seg.pt")  # load an official model

img_path = "Test_Image.png"

img = cv2.imread(img_path)



# Run YOLOv8 inference on the frame
results = model(img)

# Visualize the results ont he frame
annotated_frame = results[0].plot()

# Display the annotated frame
cv2.imshow("YOLOv8 Inference", annotated_frame)

# Break the loop when 'q' is pressed
cv2.waitKey(0)
cv2.destroyAllWindows()
