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

        # Load the YOLOv8 model
        self.model = YOLO(BASEPATH / "runs/detect/train8/weights/last.pt")

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

        # Draw bounding boxes and labels on the image
        annotated_img = results[0].plot()  # Plot the detections

        try:
            # Convert OpenCV image back to ROS Image message
            image_msg = self.bridge.cv2_to_imgmsg(annotated_img, "bgr8")
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
