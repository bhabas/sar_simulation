from ultralytics import YOLO
from pathlib import Path

BASEPATH = Path("/home/bhabas/catkin_ws/src/sar_simulation/sar_projects/Targeted_Landing")

model = YOLO(BASEPATH / "runs/detect/train8/weights/last.pt")

results = model([BASEPATH / "Test_Image.jpg"])

# Process results list
for result in results:
    boxes = result.boxes  # Boxes object for bounding box outputs
    masks = result.masks  # Masks object for segmentation masks outputs
    keypoints = result.keypoints  # Keypoints object for pose outputs
    probs = result.probs  # Probs object for classification outputs
    obb = result.obb  # Oriented boxes object for OBB outputs
    result.show()  # display to screen
    result.save(filename="result.jpg")  # save to disk