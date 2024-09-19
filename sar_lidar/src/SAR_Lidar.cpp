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
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common.h>
// Include PointCloud2 message

// Custom Messages
#include <sar_msgs/BoundingBox.h>
#include <sar_msgs/BoundingBoxArray.h>



// ROS Publisher
ros::Publisher pub;
ros::Publisher pub_bounding_boxes;


void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    // CONVERT PC2 TO PCL OBJECT
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);

    // DOWNSAMPLE
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    vg.setInputCloud(cloud);
    vg.setLeafSize(0.1,0.1,0.1);
    vg.filter(*cloud_filtered);

    // CREATE SEGMENTATION OBJECTS
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::ExtractIndices<pcl::PointXYZ> extract_indices;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::ExtractIndices<pcl::PointXYZ> extract;

    // Create a point cloud to store the vertical planes
    pcl::PointCloud<pcl::PointXYZ>::Ptr vertical_planes_accumulated(new pcl::PointCloud<pcl::PointXYZ>);


    // Configure RANSAC to find vertical planes
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(500);
    seg.setDistanceThreshold(0.02);
    seg.setAxis(Eigen::Vector3f(1,0,0));
    seg.setEpsAngle(10.0* (M_PI/180.0)); // 10 degrees

    // Prepare BoundingBoxArray message
    sar_msgs::BoundingBoxArray bbox_array;
    bbox_array.header = msg->header;

    int i = 0;
    while (cloud->points.size() > 0)
    {
        
        // Perform RANSAC to find vertical planes
        seg.setInputCloud(cloud_filtered);
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.size() == 0)
        {
            std::cout << "No more vertical planes found" << std::endl;
            break;  // Exit loop if no more vertical planes found
        }

        // Extract the inliers (the vertical surface)
        pcl::PointCloud<pcl::PointXYZ>::Ptr vertical_plane(new pcl::PointCloud<pcl::PointXYZ>);
        extract.setInputCloud(cloud_filtered);
        extract.setIndices(inliers);
        extract.setNegative(false); // Extract the inliers
        extract.filter(*vertical_plane);

        // Add the vertical plane to the accumulated point cloud
        *vertical_planes_accumulated += *vertical_plane;

        // Find points that cluster the vertical plane
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(vertical_plane);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(0.3); // 0.5 meters
        ec.setMinClusterSize(50);
        ec.setMaxClusterSize(25000);
        ec.setSearchMethod(tree);
        ec.setInputCloud(vertical_plane);
        ec.extract(cluster_indices);


        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
        for(auto it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
        {
            for(auto pit = it->indices.begin(); pit != it->indices.end(); ++pit)
            {
                cluster->points.push_back(vertical_plane->points[*pit]);
            }
        }
        cluster->width = cluster->points.size();
        cluster->height = 1;
        cluster->is_dense = true;
        
        // Compute and save the bounding box or min/max points
        pcl::PointXYZ min_pt, max_pt;
        pcl::getMinMax3D<pcl::PointXYZ>(*cluster, min_pt, max_pt);

        // Create a BoundingBox message
        sar_msgs::BoundingBox bbox;
        bbox.min_point.x = min_pt.x;
        bbox.min_point.y = min_pt.y;
        bbox.min_point.z = min_pt.z;
        bbox.max_point.x = max_pt.x;
        bbox.max_point.y = max_pt.y;
        bbox.max_point.z = max_pt.z;

        bbox_array.boxes.push_back(bbox);

        // Remove the inliers from the point cloud
        extract.setNegative(true); // Extract the outliers
        extract.filter(*cloud_filtered); // Update the point cloud without the vertical plane

        i++;
    }
       
    // Publish the BoundingBoxArray message
    pub_bounding_boxes.publish(bbox_array);


    // Convert to ROS data type and publish
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*vertical_planes_accumulated, output);
    output.header.frame_id = msg->header.frame_id;
    output.header.stamp = ros::Time::now();

    pub.publish(output);

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_processor");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/SAR_Internal/lidar/raw", 1, cloud_cb, ros::TransportHints().tcpNoDelay());

    pcl::console::setVerbosityLevel(pcl::console::L_ERROR);

    pub = nh.advertise<sensor_msgs::PointCloud2>("SAR_Internal/lidar/processed", 1);
    pub_bounding_boxes = nh.advertise<sar_msgs::BoundingBoxArray>("SAR_Internal/lidar/bounding_boxes", 1);
    std::cout << "Node Running" << std::endl;
    ros::spin();




  return 0;
}
