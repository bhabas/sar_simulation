#!/usr/bin/env python

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from ultralytics import YOLO
from sar_msgs.msg import BoundingBox, BoundingBoxArray

from pathlib import Path
BASEPATH = Path("/home/bhabas/catkin_ws/src/sar_simulation/sar_camera")


class YOLOv8Node:
    def __init__(self):
        rospy.init_node('camera_detection_node', anonymous=True)
        self.bridge = CvBridge()

        # Load custom YOLOv8 model
        self.detector = YOLO(BASEPATH / "runs/detect/train4/weights/last.pt",verbose=False)


        self.image_sub = rospy.Subscriber("/SAR_Internal/camera/image_raw", Image, self.image_callback)
        self.image_pub = rospy.Publisher("/SAR_Internal/camera/image_processed", Image, queue_size=1)
        self.bbox_pub = rospy.Publisher("/SAR_Internal/camera/bounding_boxes", BoundingBoxArray, queue_size=1)

    def image_callback(self, data):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge Error: {e}")
            return

        # Perform Inference
        detections = self.detector(cv_image)[0]

        bbox_array = BoundingBoxArray()
        bbox_array.header = data.header
       

        for box in detections.boxes:
            class_name = self.detector.names[int(box.cls)]
            if class_name in ['Burger_Sign', 'Gas_Sign'] and box.conf > 0.8:

                ## CREATE BOUDNING BOX MESSAGE
                bbox = BoundingBox()
                bbox.class_name = class_name
                bbox.confidence = box.conf.item()
                bbox.min_point.x = box.xyxy[0][0].int().item()
                bbox.min_point.y = box.xyxy[0][1].int().item()
                bbox.max_point.x = box.xyxy[0][2].int().item()
                bbox.max_point.y = box.xyxy[0][3].int().item()

                bbox_array.boxes.append(bbox)

                ## DRAW BOUNDING BOX
                cv_image = cv2.rectangle(img=cv_image, 
                                        pt1=box.xyxy[0][0:2].int().tolist(),
                                        pt2=box.xyxy[0][2:4].int().tolist(),
                                        color=(255,0,0),
                                        thickness=2)
                
                cv_image = cv2.putText(img=cv_image, 
                                    text=f"{class_name}: {box.conf.tolist()[0]:.2f}", 
                                    org=(box.xyxy[0][0].int().item(), box.xyxy[0][1].int().item()-10), 
                                    fontFace=cv2.FONT_HERSHEY_SIMPLEX, 
                                    fontScale=0.5, 
                                    color=(255,0,0), 
                                    thickness=2)

        self.bbox_pub.publish(bbox_array)



        try:
            # Convert OpenCV image back to ROS Image message
            image_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            # Publish the processed image
            self.image_pub.publish(image_msg)
        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge Error: {e}")

if __name__ == '__main__':
    try:
        yolo_node = YOLOv8Node()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
