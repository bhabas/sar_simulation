#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

// Include pcl
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/point_cloud.h>
#include <pcl/filters/extract_indices.h>

// Include PointCloud2 message



// ROS Publisher
ros::Publisher pub;


void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    // CONVERT PC2 TO PCL OBJECT
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);

    // DOWNSAMPLE
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    vg.setInputCloud(cloud);
    vg.setLeafSize(0.5f, 0.5f, 0.5f);
    vg.filter(*cloud_filtered);

    // GROUND REMOVAL
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setAxis(Eigen::Vector3f(0,0,1)); // Z up
    seg.setEpsAngle(15.0* (M_PI/180.0)); // 15 degrees
    seg.setDistanceThreshold(0.2);
    seg.setInputCloud(cloud_filtered);
    seg.segment(*inliers, *coefficients);

    // EXTRACT NON-GROUND POINTS
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud_filtered);
    extract.setIndices(inliers);
    extract.setNegative(true);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_no_ground(new pcl::PointCloud<pcl::PointXYZ>);
    extract.filter(*cloud_no_ground);


    // // VERTICAL PLANE SEGMENTATION
    // pcl::SACSegmentation<pcl::PointXYZ> vertical_seg;
    // vertical_seg.setOptimizeCoefficients(true);
    // vertical_seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    // vertical_seg.setMethodType(pcl::SAC_RANSAC);
    // vertical_seg.setMaxIterations(1000);
    // vertical_seg.setDistanceThreshold(0.02);
    // vertical_seg.setAxis(Eigen::Vector3f(1,0,0)); // Z up
    // vertical_seg.setEpsAngle(15.0* (M_PI/180.0)); // 15 degrees

    // pcl::ModelCoefficients::Ptr vertical_coefficients(new pcl::ModelCoefficients);
    // pcl::PointIndices::Ptr vertical_inliers(new pcl::PointIndices);
    // vertical_seg.setInputCloud(cloud_no_ground);
    // vertical_seg.segment(*vertical_inliers, *vertical_coefficients);

    // if (vertical_inliers->indices.size() == 0)
    // {
    //     ROS_WARN("No vertical planes foung");
    // }

    // // Extract vertical plane
    // pcl::PointCloud<pcl::PointXYZ>::Ptr vertical_plane(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::ExtractIndices<pcl::PointXYZ> extract_vertical;
    // extract_vertical.setInputCloud(cloud_no_ground);
    // extract_vertical.setIndices(vertical_inliers);
    // extract_vertical.setNegative(false);
    // extract_vertical.filter(*vertical_plane);

    // Convert to ROS data type and publish
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud_no_ground, output);
    output.header.frame_id = msg->header.frame_id;
    output.header.stamp = ros::Time::now();

    pub.publish(output);

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_processor");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/SAR_Internal/lidar/raw", 1, cloud_cb);

    pub = nh.advertise<sensor_msgs::PointCloud2>("SAR_Internal/lidar/processed", 1);
    std::cout << "Node Running" << std::endl;
    ros::spin();




  return 0;
}
