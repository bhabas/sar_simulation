#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header
import pcl
import numpy as np

from pathlib import Path
BASEPATH = Path("/home/bhabas/catkin_ws/src/sar_simulation/sar_projects/Targeted_Landing")


class LidarPreproccessNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('PointCloud_PreProcessing', anonymous=True)

        # Subscribe to the image topic
        self.lidar_sub = rospy.Subscriber("/SAR_Internal/lidar/raw", PointCloud2, self.lidar_callback)



        # Publisher for the processed image
        self.lidar_pub = rospy.Publisher("/SAR_Internal/lidar/preprocessed", PointCloud2, queue_size=1)

    def lidar_callback(self,data):
        pcl_data = self.ros_to_pcl(data)

        # Create a segmentation object
        seg = pcl_data.make_segmenter()
        seg.set_model_type(pcl.SACMODEL_PLANE)
        seg.set_method_type(pcl.SAC_RANSAC)
        seg.set_distance_threshold(0.01)

        inliers, coefficients = seg.segment()

        # Extract non-ground points
        cloud_filtered = pcl_data.extract(inliers, negative=True)
        ros_cloud_filtered = self.pcl_to_ros(cloud_filtered)

        self.lidar_pub.publish(ros_cloud_filtered)

    def ros_to_pcl(self,data):

        points_list = []

        for data in pc2.read_points(data,skip_nans=True):
            points_list.append([data[0],data[1],data[2]])

        pcl_data = pcl.BasePointCloud()
        pcl.
        pcl_data.from_list(points_list)

        return pcl_data
    
    def pcl_to_ros(self,pcl_data):

        ros_msg = PointCloud2()
        ros_msg.header = Header()
        ros_msg.header.stamp = rospy.Time.now()
        ros_msg.header.frame_id = "velodyne"
        ros_msg.height = 1
        ros_msg.width = len(pcl_data)
        ros_msg.fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1)
        ]
        ros_msg.is_bigendian = False
        ros_msg.point_step = 12




        

if __name__ == '__main__':
    try:
        LidarPreproccessNode = LidarPreproccessNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
