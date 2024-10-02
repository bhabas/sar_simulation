from ultralytics import YOLO

from ultralytics.data.converter import convert_coco

# convert_coco(labels_dir='/home/bhabas/catkin_ws/src/sar_simulation/sar_camera/datasets/Landing_Locations.v1i.coco-segmentation/valid', use_segments=True)

# model = YOLO('yolov8n-seg.pt')

# results = model.train(
#     data='/home/bhabas/catkin_ws/src/sar_simulation/sar_camera/datasets/Landing_Locations_Dataset/data.yaml',
#     epochs=40,
#     imgsz=640,
# )


model = YOLO("/home/bhabas/catkin_ws/src/sar_simulation/runs/segment/train/weights/best.pt")
results = model(["/home/bhabas/catkin_ws/src/sar_simulation/sar_general/Scripts/frames/frame_0172.jpg"])

for result in results:
    boxes = result.boxes  # Boxes object for bounding box outputs
    masks = result.masks  # Masks object for segmentation masks outputs
    keypoints = result.keypoints  # Keypoints object for pose outputs
    probs = result.probs  # Probs object for classification outputs
    obb = result.obb  # Oriented boxes object for OBB outputs
    result.show()  # display to screen
    result.save(filename="result.jpg")  # save to disk