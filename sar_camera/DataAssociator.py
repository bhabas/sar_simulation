#!/usr/bin/env python

import rospy
import tf2_ros
import tf2_geometry_msgs
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from sar_msgs.msg import BoundingBoxArray, BoundingBox
from sar_msgs.msg import LandingTarget, LandingTargetArray
from geometry_msgs.msg import Point, PointStamped, PoseStamped
import ros_numpy

from message_filters import Subscriber, ApproximateTimeSynchronizer

class DataAssociator:
    def __init__(self):
        rospy.init_node('DataAssociator_Node', anonymous=True)
        self.bridge = CvBridge()

        # Store latest image for projection if needed
        self.latest_image = None
        self.image_received = False


        # Subscribers 
        self.lidar_bbox_sub = Subscriber('/SAR_Internal/lidar/bounding_boxes_raw', BoundingBoxArray)
        self.camera_bbox_sub = Subscriber('/SAR_Internal/camera/bounding_boxes', BoundingBoxArray)
        self.image_sub = rospy.Subscriber('/SAR_Internal/camera/image_raw', Image, self.camera_callback,queue_size=1)

        # Publishers
        self.image_pub = rospy.Publisher("/SAR_Internal/camera/image_processed_bbox", Image, queue_size=1)
        self.lidar_bbox_pub = rospy.Publisher('/SAR_Internal/lidar/bounding_boxes_processed', BoundingBoxArray, queue_size=1)
        self.LandingTarget_pub = rospy.Publisher('/LandingTargets', LandingTargetArray, queue_size=1)


        queue_size = 10
        max_delay = 0.05
        self.time_sync = ApproximateTimeSynchronizer([self.lidar_bbox_sub, self.camera_bbox_sub],queue_size, max_delay)
        self.time_sync.registerCallback(self.AssociateTargets)


        # TF Buffer and Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Camera Intrinsic Parameters
        self.K = np.array([[381.362, 0, 320.5], 
                           [0, 381.362, 320.5], 
                           [0, 0, 1]])
        
        self.cam_bboxes = []
        self.lidar_bboxes = []
        
    def AssociateTargets(self, lidar_bbox_array_msg, camera_bbox_array_msg):

        matched_targets = []

        # Get transform from Lidar frame to Camera frame
        transform = self.get_transform(self.tf_buffer, 'camera_link_optical', 'velodyne_base_link')
        if transform is None:
            return

        self.cam_bboxes = camera_bbox_array_msg.boxes
        self.lidar_bboxes = lidar_bbox_array_msg.boxes



        for cam_bbox in self.cam_bboxes:
            
            best_iou = 0
            best_lidar_bbox = None

            for lidar_bbox in self.lidar_bboxes:
                
                lidar_bbox_tf = BoundingBox()
                lidar_bbox_tf.class_name = lidar_bbox.class_name
                lidar_bbox_tf.confidence = lidar_bbox.confidence

                # Transform bounding box to camera frame
            
                # Transform bounding box to image coordinates
                lidar_bbox_tf.min_point = self.project_to_image(self.transform(lidar_bbox.max_point, transform))
                lidar_bbox_tf.max_point = self.project_to_image(self.transform(lidar_bbox.min_point, transform))

                # Skip if projection failed
                if lidar_bbox_tf.min_point.x is None or lidar_bbox_tf.max_point.x is None:
                    continue 
          
                iou = self.calculate_iou(cam_bbox, lidar_bbox_tf)

                if iou > best_iou:
                    best_iou = iou
                    best_lidar_bbox_tf = lidar_bbox_tf
                    best_lidar_bbox = lidar_bbox

            if best_iou > 0.3: # IoU threshold
                rospy.loginfo(f"Match Found:  IoU: {best_iou:.3f}")
                # rospy.loginfo(f"Camera BBox: {cam_bbox.min_point} - {cam_bbox.max_point}")
                # rospy.loginfo(f"Lidar BBox: {best_lidar_bbox.min_point} - {best_lidar_bbox.max_point}")

                matched_target = LandingTarget()
                matched_target.BBox_Cam.class_name = cam_bbox.class_name
                matched_target.BBox_Cam.confidence = cam_bbox.confidence
                matched_target.BBox_Cam.min_point = cam_bbox.min_point
                matched_target.BBox_Cam.max_point = cam_bbox.max_point


                matched_target.BBox_Lidar.class_name = best_lidar_bbox.class_name
                matched_target.BBox_Lidar.min_point = best_lidar_bbox.min_point
                matched_target.BBox_Lidar.max_point = best_lidar_bbox.max_point

                matched_target.Pose_Centroid.position = self.compute_center(best_lidar_bbox)
                matched_targets.append(matched_target)

   
                # Draw bounding box on image
                min_pt = ros_numpy.numpify(cam_bbox.min_point)[:2].astype(int)
                max_pt = ros_numpy.numpify(cam_bbox.max_point)[:2].astype(int)
                self.latest_image = cv2.rectangle(img=self.latest_image, 
                                        pt1=(min_pt[0], min_pt[1]),
                                        pt2=(max_pt[0], max_pt[1]),
                                        color=(255,0,0),
                                        thickness=2)
            
                

        # Publish matched targets
        if matched_targets:
            matched_targets_msg = LandingTargetArray()
            matched_targets_msg.LandingTargets = matched_targets
            matched_targets_msg.header = camera_bbox_array_msg.header
            self.LandingTarget_pub.publish(matched_targets_msg)


            # Publish new bounding lidar box onto image
            image_msg = self.bridge.cv2_to_imgmsg(self.latest_image, "bgr8")
            self.image_pub.publish(image_msg)





    def compute_center(self, bbox):
        x_center = bbox.min_point.x
        y_center = (bbox.max_point.y + bbox.min_point.y)/2
        z_center = (bbox.max_point.z + bbox.min_point.z)/2

        return Point(x=x_center, y=y_center, z=z_center)



    def camera_callback(self, img_msg):
        self.latest_image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        self.image_received = True


    def calculate_iou(self, boxA, boxB):

        boxA_area = abs(boxA.max_point.x - boxA.min_point.x) * abs(boxA.max_point.y - boxA.min_point.y)
        boxB_area = abs(boxB.max_point.x - boxB.min_point.x) * abs(boxB.max_point.y - boxB.min_point.y)

        # Determine the (x, y)-coordinates of the intersection rectangle
        xA = max(boxA.min_point.x, boxB.min_point.x)
        yA = max(boxA.min_point.y, boxB.min_point.y)
        xB = min(boxA.max_point.x, boxB.max_point.x)
        yB = min(boxA.max_point.y, boxB.max_point.y)

        # Compute the area of intersection rectangle
        interWidth = abs(xB - xA)
        interHeight = abs(yB - yA)

        interArea = interWidth * interHeight

        # Compute the IoU
        iou = interArea / float(boxA_area + boxB_area - interArea)
        return iou

       
        
    
    def transform(self, point, transform):

        point_stamped = PointStamped()
        point_stamped.point = point
        point_transformed = tf2_geometry_msgs.do_transform_point(point_stamped, transform).point
        return point_transformed
        
    
    def project_to_image(self, point_cam):

        point_cam = ros_numpy.numpify(point_cam)
        # Project a 3D point in camera frame to 2D image coordinates
        if point_cam[2] < 0:
            # rospy.logwarn("Point behind camera. Skipping projection.")
            point_cam = Point()
            point_cam.x = None
            point_cam.y = None
            return point_cam
        
        point_2d = np.dot(self.K, point_cam)
        x = point_2d[0] / point_2d[2]
        y = point_2d[1] / point_2d[2]

        point_cam = Point()
        point_cam.x = int(x)
        point_cam.y = int(y)
        return point_cam
        

    
            
    def get_transform(self, tf_buffer, target_frame, source_frame):
        try:
            transform = tf_buffer.lookup_transform(target_frame, source_frame, rospy.Time(0), rospy.Duration(1.0))
            return transform
        
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("Transform from %s to %s not available.", source_frame, target_frame)
            return None
    

if __name__ == '__main__':
    try:
        transformer = DataAssociator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass