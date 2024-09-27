#!/usr/bin/env python

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from ultralytics import YOLO
from sar_msgs.msg import BoundingBox, BoundingBoxArray
from message_filters import Subscriber, ApproximateTimeSynchronizer
import numpy as np
from stereo_msgs.msg import DisparityImage   

from pathlib import Path
BASEPATH = Path("/home/bhabas/catkin_ws/src/sar_simulation/sar_camera")


class CamProcessor:
    def __init__(self):
        rospy.init_node('camera_detection_node', anonymous=True)
        self.bridge = CvBridge()

        # Load custom YOLOv8 model
        self.detector = YOLO(BASEPATH / "runs/detect/train4/weights/last.pt",verbose=False)

        self.image_sub = Subscriber("/stereo/left/image_rect_color", Image)
        self.disparity_sub = Subscriber("/stereo/disparity", DisparityImage)

        self.image_pub = rospy.Publisher("/stereo/left/image_rect_color/image_processed", Image, queue_size=1)
        self.bbox_pub = rospy.Publisher("/SAR_Internal/camera/bounding_boxes", BoundingBoxArray, queue_size=1)

        queue_size = 10
        max_delay = 0.1
        self.time_sync = ApproximateTimeSynchronizer([self.image_sub, self.disparity_sub],queue_size, max_delay)
        self.time_sync.registerCallback(self.image_callback)

    def image_callback(self, img_msg, disparity_msg):

        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
            self.disparity_image = self.bridge.imgmsg_to_cv2(disparity_msg.image, "32FC1")
        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge Error: {e}")
            return

        # Perform Inference
        detections = self.detector(cv_image)[0]

        bbox_array = BoundingBoxArray()
        bbox_array.header = img_msg.header
       

        for box in detections.boxes:
            class_name = self.detector.names[int(box.cls)]
            if class_name in ['Burger_Sign', 'Gas_Sign'] and box.conf > 0.8:

                x1, y1, x2, y2 = box.xyxy.flatten().int().tolist()

                disparity = self.get_median_disparity(x1, y1, x2, y2)
                if disparity is None:
                    rospy.logwarn("Invalid disparity for bounding box.")
                    continue
                
                f = 381.362
                B = 0.2
                Z = f * B / disparity

                ## CREATE BOUDNING BOX MESSAGE
                bbox = BoundingBox()
                bbox.class_name = class_name
                bbox.confidence = box.conf.item()
                bbox.min_point.x = x1
                bbox.min_point.y = y1
                bbox.min_point.z = Z
                bbox.max_point.x = x2
                bbox.max_point.y = y2
                bbox.max_point.z = Z

                bbox_array.boxes.append(bbox)

                print(f"{class_name}: {box.conf.tolist()[0]:.2f} | Disparity: {disparity:.2f} | Depth: {Z:.2f}")

                ## DRAW BOUNDING BOX
                cv_image = cv2.rectangle(img=cv_image, 
                                        pt1=tuple(map(int, box.xyxy.flatten().tolist()[:2])),
                                        pt2=tuple(map(int, box.xyxy.flatten().tolist()[2:])),
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

    def get_median_disparity(self, x1, y1, x2, y2):
        disparity_roi = self.disparity_image[y1:y2, x1:x2]
        valid_disparity = disparity_roi[disparity_roi > 0]
        if len(valid_disparity) == 0:
            return None
        return np.median(valid_disparity)

if __name__ == '__main__':
    try:
        yolo_node = CamProcessor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
