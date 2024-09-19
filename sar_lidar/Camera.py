import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from sklearn.cluster import DBSCAN

image_width = 640
horizontal_fov = 1.4
fx = 381.36246688113556
fy = fx

cx = 320.5
cy = 320.5


class CameraProcessor:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/SAR_Internal/camera/image_raw', Image, self.image_callback)

        self.edges_pub = rospy.Publisher('/SAR_Internal/camera/image_edges', Image, queue_size=10)
        self.lines_pub = rospy.Publisher('/SAR_Internal/camera/image_lines', Image, queue_size=10)


    def image_callback(self, msg):

        # Convert ROS Image message to OpenCV image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return
        
        # Proceed with image processing
        self.process_image(cv_image)

    def process_image(self, cv_image):

        # Step 1: Undistort the image
        undistorted = cv_image

        # Step 2: Apply edge detection
        edges = self.detect_edges(undistorted)
        self.publish_image(self.edges_pub, edges, encoding="mono8")

        # Step 3: Detect and filter vertical lines
        lines = self.detect_lines(edges)
        vertical_lines = self.filter_vertical_lines(lines)

        # Draw the lines on the original image
        image_with_lines = cv_image.copy()
        # if vertical_lines:
        #     for (x1, y1, x2, y2) in vertical_lines:
        #         cv2.line(image_with_lines, (x1, y1), (x2, y2), (0, 255, 0), 2)

        self.publish_image(self.lines_pub, image_with_lines)

        # Step 4: Cluster the lines
        clustered_lines = self.cluster_lines(vertical_lines)

        # Draw the clustered lines on the original image
        image_with_clusters = cv_image.copy()
        if clustered_lines:
            for cluster in clustered_lines.values():
                for (x1, y1, x2, y2) in cluster:
                    cv2.line(image_with_clusters, (x1, y1), (x2, y2), (0, 0, 255), 2)

        # self.publish_image(self.lines_pub, image_with_clusters)





    def detect_edges(self, image):
        # Convert to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Apply Gaussian blur to reduce noise and improve edge detection
        # blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        blurred = gray

        # Apply Canny edge detection
        edges = cv2.Canny(blurred, threshold1=50, threshold2=150)
        return edges
    
    def detect_lines(self, edges):
        # Use Standard Hough Line Transform
        lines = cv2.HoughLines(
            edges,
            rho=1,
            theta=np.pi / 180,
            threshold=150  # Adjust threshold based on your data
        )
        return lines
    
    def filter_vertical_lines(self, lines):
        vertical_lines = []

        if lines is not None:
            for line in lines:
                for rho, theta in line:
                    # Convert polar coordinates (rho, theta) to Cartesian coordinates
                    a = np.cos(theta)
                    b = np.sin(theta)
                    x0 = a * rho
                    y0 = b * rho
                    # Generate two points to represent the line
                    x1 = int(x0 + 1000 * (-b))
                    y1 = int(y0 + 1000 * (a))
                    x2 = int(x0 - 1000 * (-b))
                    y2 = int(y0 - 1000 * (a))
                    
                    # Calculate the angle of the line in degrees
                    angle = np.abs(theta * 180.0 / np.pi)
                    
                    # Check if the angle is close to vertical (near 90 degrees)
                    if 80 < angle < 100:
                        vertical_lines.append((x1, y1, x2, y2))

        return vertical_lines

    

    def cluster_lines(self, vertical_lines):

        # Convert lines to midpoints for clustering
        line_points = []
        for x1, y1, x2, y2 in vertical_lines:
            mid_x = (x1 + x2) / 2
            mid_y = (y1 + y2) / 2
            line_points.append([mid_x, mid_y])

        line_points = np.array(line_points)

        # Cluster the midpoints using DBSCAN
        clustering = DBSCAN(eps=10, min_samples=2).fit(line_points)
        labels = clustering.labels_

        # Group the lines based on the cluster labels
        clustered_lines = {}
        for idx, label in enumerate(labels):
            if label == -1:
                continue # Noise
            if label not in clustered_lines:
                clustered_lines[label] = []
            clustered_lines[label].append(vertical_lines[idx])

        return clustered_lines
    
    def publish_image(self, publisher, image, encoding="bgr8"):
        """
        Helper function to convert an OpenCV image to a ROS Image message and publish it.
        """
        try:
            ros_image = self.bridge.cv2_to_imgmsg(image, encoding)
            publisher.publish(ros_image)
        except CvBridgeError as e:
            rospy.logerr(f"Failed to publish image: {e}")

if __name__ == '__main__':
    rospy.init_node('camera_processor',anonymous=True)
    cp = CameraProcessor()
    rospy.spin()