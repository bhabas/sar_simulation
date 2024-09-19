#!/usr/bin/env python

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from ultralytics import YOLO

from pathlib import Path
BASEPATH = Path("/home/bhabas/catkin_ws/src/sar_simulation/sar_projects/Targeted_Landing")


class YOLOv8Node:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('yolov8_node', anonymous=True)

        # Load custom YOLOv8 model
        self.model = YOLO(BASEPATH / "runs/detect/train4/weights/last.pt",verbose=False)

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Subscribe to the image topic
        self.image_sub = rospy.Subscriber("/SAR_Internal/camera/image_raw", Image, self.image_callback)

        # Publisher for the processed image
        self.image_pub = rospy.Publisher("/camera/image_yolov8", Image, queue_size=1)

    def image_callback(self, data):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge Error: {e}")
            return

        # Run the YOLO model on the image
        results = self.model(cv_image)

        landing_site_dict = {}
        id_dict = {0: "Landing_Site_1", 1:"Landing_Site_2"}


        ## Find Viable Landing Sites
        for box in results[0].boxes:

            if box.conf.tolist()[0] > 0.8:

                landing_site_dict[0] = {"id": box.cls.tolist()[0], "box": box.xyxy[0].int().tolist()}


                cv_image = cv2.rectangle(img=cv_image, 
                                        pt1=box.xyxy[0][0:2].int().tolist(),
                                        pt2=box.xyxy[0][2:4].int().tolist(),
                                        color=(255,0,0),
                                        thickness=2)
                cv_image = cv2.putText(img=cv_image, 
                                    text=f"{id_dict[box.cls.tolist()[0]]}: {box.conf.tolist()[0]:.2f}", 
                                    org=(box.xyxy[0][0].int().item(), box.xyxy[0][1].int().item()-10), 
                                    fontFace=cv2.FONT_HERSHEY_SIMPLEX, 
                                    fontScale=0.5, 
                                    color=(255,0,0), 
                                    thickness=2)


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
