import rospy
import tf2_ros
import tf2_geometry_msgs
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from sar_msgs.msg import BoundingBoxArray, BoundingBox
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


        queue_size = 10
        max_delay = 0.05
        self.time_sync = ApproximateTimeSynchronizer([self.lidar_bbox_sub, self.camera_bbox_sub],queue_size, max_delay)
        self.time_sync.registerCallback(self.SyncCallback)


        # TF Buffer and Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Camera Intrinsic Parameters
        self.K = np.array([[381.362, 0, 320.5], 
                           [0, 381.362, 320.5], 
                           [0, 0, 1]])
        
        self.cam_bboxes = []
        self.lidar_bboxes = []
        
    def SyncCallback(self, lidar_bbox_array_msg, camera_bbox_array_msg):

        ## Camera Bounding Boxes
        self.cam_bboxes = camera_bbox_array_msg.boxes


        ## Lidar Bounding Boxes
        
        # Get transform from Lidar frame to Camera frame
        transform = self.get_transform(self.tf_buffer, 'camera_link_optical', 'velodyne_base_link')
        if transform is None:
            return

        # Process each bounding box
        for bbox in lidar_bbox_array_msg.boxes:

            # Transform bounding box to camera frame
            max_pt_cam = self.transform(bbox.min_point, transform)
            min_pt_cam = self.transform(bbox.max_point, transform)

            # Transform bounding box to image coordinates
            min_pt_cam = self.project_to_image(min_pt_cam)
            max_pt_cam = self.project_to_image(max_pt_cam)

            # Skip if projection failed
            if min_pt_cam.x is None or max_pt_cam.x is None:
                continue 


            # Create new bounding box message
            bbox_lidar = BoundingBox()
            bbox_lidar.class_name = bbox.class_name
            bbox_lidar.confidence = bbox.confidence
            bbox_lidar.min_point = min_pt_cam
            bbox_lidar.max_point = max_pt_cam

            self.lidar_bboxes.append(bbox_lidar)


            

        for cam_bbox in self.cam_bboxes:

            cam_bbox = (int(cam_bbox.min_point.x), int(cam_bbox.min_point.y), 
                        int(cam_bbox.max_point.x), int(cam_bbox.max_point.y))
            
            best_iou = 0
            best_lidar_bbox = None

            for lidar_bbox in self.lidar_bboxes:
                lidar_bbox = (int(lidar_bbox.min_point.x), int(lidar_bbox.min_point.y),
                              int(lidar_bbox.max_point.x), int(lidar_bbox.max_point.y))
                
                iou = self.calculate_iou(cam_bbox, lidar_bbox)

                if iou > best_iou:
                    best_iou = iou
                    best_lidar_bbox = lidar_bbox

            if best_iou > 0.3: # IoU threshold
                rospy.loginfo(f"Match Found: {best_lidar_bbox} and {cam_bbox} and IoU: {best_iou}")

            # Draw bounding box on image
            self.latest_image = cv2.rectangle(img=self.latest_image, 
                                    pt1=best_lidar_bbox[0:2],
                                    pt2=best_lidar_bbox[2:4],
                                    color=(255,0,0),
                                    thickness=2)
            
            # Publish new bounding lidar box onto image
            image_msg = self.bridge.cv2_to_imgmsg(self.latest_image, "bgr8")
            self.image_pub.publish(image_msg)




        print()

    def compute_center(self, bbox):

        # x_center = (bbox.points[0].x + bbox.points[2].x) / 2
        # y_center = (bbox.points[0].y + bbox.points[2].y) / 2
        # z_center = 0  # Assuming vertical surfaces, z can be set as needed
        # return PoseStamped(
        #     header=rospy.Header(),
        #     pose=geometry_msgs.msg.Pose(
        #         position=geometry_msgs.msg.Point(x=x_center, y=y_center, z=z_center),
        #         orientation=geometry_msgs.msg.Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)
        #     )
        # )
        pass


    def camera_callback(self, img_msg):
        self.latest_image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        self.image_received = True


    def calculate_iou(self, boxA, boxB):

        # boxA and boxB are in the format [x1, y1, x2, y2]
        xA = max(boxA[0], boxB[0])
        yA = max(boxA[1], boxB[1])
        xB = min(boxA[2], boxB[2])
        yB = min(boxA[3], boxB[3])

        # Compute the area of intersection rectangle
        interWidth = max(0, xB - xA + 1)
        interHeight = max(0, yB - yA + 1)
        interArea = interWidth * interHeight

        # Compute the area of both bounding boxes
        boxAArea = (boxA[2] - boxA[0] + 1) * (boxA[3] - boxA[1] + 1)
        boxBArea = (boxB[2] - boxB[0] + 1) * (boxB[3] - boxB[1] + 1)

        # Compute the IoU
        iou = interArea / float(boxAArea + boxBArea - interArea)
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