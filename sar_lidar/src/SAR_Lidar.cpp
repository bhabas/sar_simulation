#include <ros/ros.h>

// Include pcl
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

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



    // Convert to ROS data type and publish
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud, output);
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
