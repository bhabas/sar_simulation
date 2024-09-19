import rospy
import tf2_ros
import tf2_geometry_msgs
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from sar_msgs.msg import BoundingBoxArray, BoundingBox
from geometry_msgs.msg import Point, PointStamped, PoseStamped, Vector3
from tf2_geometry_msgs import do_transform_point
import ros_numpy

class LidarToCameraTransformer:
    def __init__(self):
        rospy.init_node('lidar_to_camera_transformer', anonymous=True)
        self.bridge = CvBridge()

        # Store latest image for projection if needed
        self.latest_image = None
        self.image_received = False


        # Subscribers 
        self.lidar_sub = rospy.Subscriber('/SAR_Internal/lidar/bounding_boxes', BoundingBoxArray, self.lidar_bbox_callback,queue_size=1)
        self.image_sub = rospy.Subscriber('/SAR_Internal/camera/image_raw', Image, self.camera_callback,queue_size=1)

        # Publishers
        self.image_pub = rospy.Publisher("/SAR_Internal/camera/image_processed_bbox", Image, queue_size=1)

    
        # TF Buffer and Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Camera Intrinsic Parameters
        fx = 381.36246688113556
        fy = fx
        cx = 320.5
        cy = 320.5
        self.K = np.array([[fx, 0, cx], 
                           [0, fy, cy], 
                           [0, 0, 1]])
        

        

    def camera_callback(self, img_msg):
        self.latest_image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        self.image_received = True

    def lidar_bbox_callback(self, bbox_array_msg):
        if not self.image_received:
            rospy.logwarn("No image received yet. Skipping lidar processing.")
            return
        
        # Get transform from Lidar frame to Camera frame
        transform = get_transform(self.tf_buffer, 'camera_link_optical', 'velodyne_base_link')
        if transform is None:
            return

        for bbox in bbox_array_msg.boxes:

            # T = transform.transform
            min_pt_cam = self.transform_transform(bbox.min_point, transform)
            max_pt_cam = self.transform_transform(bbox.max_point, transform)

            # Project to image plane
            x1, y1 = self.project_to_image(min_pt_cam)
            x2, y2 = self.project_to_image(max_pt_cam)

            if x1 is None or x2 is None:
                continue # Skip if projection failed

            ## DRAW BOUNDING BOX
            cv_image = cv2.rectangle(img=self.latest_image, 
                                    pt1=(x1,y1),
                                    pt2=(x2,y2),
                                    color=(255,0,0),
                                    thickness=2)
            
            try:
                # Convert OpenCV image back to ROS Image message
                image_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
                # Publish the processed image
                self.image_pub.publish(image_msg)
            except CvBridgeError as e:
                rospy.logerr(f"CV Bridge Error: {e}")
            
            # cv_image = cv2.putText(img=cv_image, 
            #                     text=f"{class_name}: {box.conf.tolist()[0]:.2f}", 
            #                     org=(box.xyxy[0][0].int().item(), box.xyxy[0][1].int().item()-10), 
            #                     fontFace=cv2.FONT_HERSHEY_SIMPLEX, 
            #                     fontScale=0.5, 
            #                     color=(255,0,0), 
            #                     thickness=2)


        
    
    def transform_transform(self, point, transform):

        point_stamped = PointStamped()
        point_stamped.point = point
        point_transformed = do_transform_point(point_stamped, transform).point
        return ros_numpy.numpify(point_transformed)
        
    
    def project_to_image(self, point_cam):
        # Project a 3D point in camera frame to 2D image coordinates
        if point_cam[2] < 0:
            # rospy.logwarn("Point behind camera. Skipping projection.")
            return None, None
        
        point_2d = np.dot(self.K, point_cam)
        x = point_2d[0] / point_2d[2]
        y = point_2d[1] / point_2d[2]
        return int(x), int(y)
        

    
        
def get_transform(tf_buffer, target_frame, source_frame):
    try:
        transform = tf_buffer.lookup_transform(target_frame, source_frame, rospy.Time(0), rospy.Duration(1.0))
        return transform
    
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.logwarn("Transform from %s to %s not available.", source_frame, target_frame)
        return None
    

if __name__ == '__main__':
    try:
        transformer = LidarToCameraTransformer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass