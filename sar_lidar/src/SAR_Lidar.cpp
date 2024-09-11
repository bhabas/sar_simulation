#include <ros/ros.h>

// Include pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>

// Include PointCloud2 message
#include <sensor_msgs/PointCloud2.h>

static const std::string IMAGE_TOPIC = "/SAR_Internal/lidar/raw";
static const std::string PUBLISH_TOPIC = "/pcl/points";

// ROS Publisher
ros::Publisher pub;


void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *cloud);

    // Step 2: Segment the largest plane (floor) using RANSAC
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.3);  // Adjust this for your floor height tolerance
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.empty()) {
        ROS_WARN("No floor found");
        return;
    }

    // Step 3: Extract the points that are not part of the floor (remaining points)
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);  // true to remove the floor points
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    extract.filter(*cloud_filtered);




    // Convert to ROS data type and publish
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud_filtered, output);
    output.header.frame_id = cloud_msg->header.frame_id;
    output.header.stamp = ros::Time::now();

    pub.publish(output);

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_node");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe(IMAGE_TOPIC, 1, cloud_cb);

    // ros::Subscriber sub = nh.subscribe("/SAR_Internal/lidar/raw");
    pub = nh.advertise<sensor_msgs::PointCloud2>(PUBLISH_TOPIC, 1);
    ros::spin();




  return 0;
}
