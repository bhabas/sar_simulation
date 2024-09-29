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
        self.image_sub = rospy.Subscriber('/stereo/left/image_rect_color', Image, self.camera_callback,queue_size=1)

        # Publishers
        self.image_pub = rospy.Publisher("/SAR_Internal/camera/image_processed_bbox", Image, queue_size=1)
        self.lidar_bbox_pub = rospy.Publisher('/SAR_Internal/lidar/bounding_boxes_processed', BoundingBoxArray, queue_size=1)
        self.LandingTarget_pub = rospy.Publisher('/LandingTargets', LandingTargetArray, queue_size=1)


        queue_size = 10
        max_delay = 0.1
        self.time_sync = ApproximateTimeSynchronizer([self.lidar_bbox_sub, self.camera_bbox_sub],queue_size, max_delay)
        self.time_sync.registerCallback(self.AssociateTargets)


        # TF Buffer and Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Camera Intrinsic Parameters
        self.K = np.array([[381.362, 0, 320.5], 
                           [0, 381.362, 320.5], 
                           [0, 0, 1]])
        
        self.K_inv = np.linalg.inv(self.K)
        
        self.cam_bboxes_px = []
        self.lidar_bboxes_m = []
        
    def AssociateTargets(self, lidar_bbox_array_msg, camera_bbox_array_msg):

        matched_targets = []

        # Get transform from Lidar frame to Camera frame
        tf_lidar_cam = self.get_transform(self.tf_buffer, 'stereo_camera_link_optical_left', 'velodyne_base_link')
        if tf_lidar_cam is None:
            return
        
        tf_cam_body = self.get_transform(self.tf_buffer, 'SAR_Body', 'stereo_camera_link_optical_left')
        if tf_cam_body is None:
            return
        
        tf_lidar_body = self.get_transform(self.tf_buffer, 'SAR_Body', 'velodyne_base_link')
        if tf_lidar_body is None:
            return



        self.cam_bboxes_px = camera_bbox_array_msg.boxes   # Camera Frame
        self.lidar_bboxes_m = lidar_bbox_array_msg.boxes  # Lidar Frame

        # Find Corresponding Bounding Boxes
        for cam_bbox_px_CamFrame in self.cam_bboxes_px:
            
            best_iou = 0
            best_lidar_bbox_m = None

            for lidar_bbox_m_LidarFrame in self.lidar_bboxes_m:

                # Find IoU between Camera and Lidar Bounding Boxes

                # Transform Lidar Bounding Box to Camera Frame
                lidar_bbox_m_CamFrame = BoundingBox()
                lidar_bbox_m_CamFrame.min_point = self.transform(lidar_bbox_m_LidarFrame.max_point, tf_lidar_cam)
                lidar_bbox_m_CamFrame.max_point = self.transform(lidar_bbox_m_LidarFrame.min_point, tf_lidar_cam)

                # Convert to pixel values
                lidar_bbox_px_CamFrame = BoundingBox()
                lidar_bbox_px_CamFrame.min_point = self.project_to_image(lidar_bbox_m_CamFrame.min_point)
                lidar_bbox_px_CamFrame.max_point = self.project_to_image(lidar_bbox_m_CamFrame.max_point)

                # Skip if projection failed
                if lidar_bbox_px_CamFrame.min_point.x is None or lidar_bbox_px_CamFrame.max_point.x is None:
                    continue 
          
                iou = self.calculate_iou(cam_bbox_px_CamFrame, lidar_bbox_px_CamFrame)

                if iou > best_iou:
                    best_iou = iou
                    best_lidar_bbox_px_CamFrame = lidar_bbox_px_CamFrame
                    best_lidar_bbox_m_CamFrame = lidar_bbox_m_CamFrame

            if best_iou > 0.3: # IoU threshold
                rospy.loginfo(f"Match Found:  IoU: {best_iou:.3f}")
                # rospy.loginfo(f"Camera BBox: {cam_bbox.min_point} - {cam_bbox.max_point}")
                # rospy.loginfo(f"Lidar BBox: {best_lidar_bbox.min_point} - {best_lidar_bbox.max_point}")

                matched_target = LandingTarget()
                matched_target.BBox_Cam_px = cam_bbox_px_CamFrame
                matched_target.BBox_Lidar_px = best_lidar_bbox_px_CamFrame


                # Camera Bounding Box in meters
                matched_target.BBox_Cam_m.class_name = cam_bbox_px_CamFrame.class_name
                matched_target.BBox_Cam_m.confidence = cam_bbox_px_CamFrame.confidence

                Z_cam_depth_m = cam_bbox_px_CamFrame.min_point.z
                X_min, Y_min, Z_min = Z_cam_depth_m * self.K_inv @ np.array([cam_bbox_px_CamFrame.min_point.x, cam_bbox_px_CamFrame.min_point.y, 1])
                X_max, Y_max, Z_max = Z_cam_depth_m * self.K_inv @ np.array([cam_bbox_px_CamFrame.max_point.x, cam_bbox_px_CamFrame.max_point.y, 1])
                matched_target.BBox_Cam_m.min_point = Point(x=X_min, y=Y_min, z=Z_min)
                matched_target.BBox_Cam_m.max_point = Point(x=X_max, y=Y_max, z=Z_max)
                
                matched_target.Pose_Centroid_Cam_body.position =  self.transform(self.compute_center(matched_target.BBox_Cam_m), tf_cam_body)

                # Lidar Bounding Box in Body Frame
                matched_target.BBox_Lidar_m = best_lidar_bbox_m_CamFrame
                matched_target.Pose_Centroid_Lidar_body.position = self.transform(self.compute_center(matched_target.BBox_Lidar_m),tf_cam_body)

                matched_targets.append(matched_target)

   
                # Draw bounding box on image
                min_pt = ros_numpy.numpify(cam_bbox_px_CamFrame.min_point)[:2].astype(int)
                max_pt = ros_numpy.numpify(cam_bbox_px_CamFrame.max_point)[:2].astype(int)
                self.latest_image = cv2.rectangle(img=self.latest_image, 
                                        pt1=(min_pt[0], min_pt[1]),
                                        pt2=(max_pt[0], max_pt[1]),
                                        color=(255,0,0),
                                        thickness=2)
            
                

        # Publish matched targets
        if matched_targets:
            matched_targets_msg = LandingTargetArray()
            matched_targets_msg.LandingTargets = matched_targets
            matched_targets_msg.header.stamp = camera_bbox_array_msg.header.stamp
            matched_targets_msg.header.frame_id = 'SAR_Body'
            self.LandingTarget_pub.publish(matched_targets_msg)


            # Publish new bounding lidar box onto image
            image_msg = self.bridge.cv2_to_imgmsg(self.latest_image, "bgr8")
            self.image_pub.publish(image_msg)





    def compute_center(self, bbox):
        x_center = (bbox.min_point.x + bbox.max_point.x)/2
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