from ultralytics import YOLO


# Load a pretrained YOLO model (recommended for training)
model = YOLO("yolov8s.pt")

results = model.train(
    data="/home/bhabas/catkin_ws/src/sar_simulation/sar_projects/Targeted_Landing/datasets/Landing_Location_Dataset/data.yaml", 
    epochs=250, 
    imgsz=640)

# model = YOLO("/home/bhabas/catkin_ws/src/sar_simulation/runs/detect/train7/weights/last.pt")
# results = model.train(epochs=50)


# from ultralytics import settings

# # View all settings
# print(settings)

# # Return a specific setting
# value = settings["runs_dir"]