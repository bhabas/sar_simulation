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
ros::Publisher pub_processed_cloud;
ros::Publisher pub_bounding_boxes;


void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    // CONVERT PC2 TO PCL OBJECT
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg, *cloud);

    std::cout << "Time: " << msg->header.stamp << std::endl;

    // DOWNSAMPLE
    // pcl::VoxelGrid<pcl::PointXYZI> vg;
    // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    // vg.setInputCloud(cloud);
    // vg.setLeafSize(0.1,0.1,0.1);
    // vg.filter(*cloud_filtered);

    // CREATE SEGMENTATION OBJECTS
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZI> seg;

    // Configure RANSAC to find vertical planes
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setAxis(Eigen::Vector3f(1,0,0));
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(0.3);
    seg.setEpsAngle(5.0*(M_PI/180.0)); // 10 degrees
    

    pcl::ExtractIndices<pcl::PointXYZI> extract_indices;
    

    // Create a point cloud to store the vertical planes
    pcl::PointCloud<pcl::PointXYZI>::Ptr vertical_planes_accumulated(new pcl::PointCloud<pcl::PointXYZI>);


    // Prepare BoundingBoxArray message
    sar_msgs::BoundingBoxArray bbox_array;
    bbox_array.header = msg->header;

    int i = 0;
    while (cloud->points.size() > 0)
    {
        
        // Perform RANSAC to find vertical planes
        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.size() == 0)
        {
            std::cout << "No more vertical planes found" << std::endl;
            break;  // Exit loop if no more vertical planes found
        }

        // Extract the inliers from input cloud
        pcl::PointCloud<pcl::PointXYZI>::Ptr vertical_plane(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::ExtractIndices<pcl::PointXYZI> extract;
        extract.setInputCloud(cloud);
        extract.setIndices(inliers);
        extract.setNegative(false); // Extract the inliers

        // Get the points associated with planar surface
        extract.filter(*vertical_plane);

        // Add the vertical plane to the accumulated point cloud
        *vertical_planes_accumulated += *vertical_plane;

        // for (const auto& point : vertical_plane->points)
        // {
        //     std::cout << "Point (x, y, z): " << point.x << ", " << point.y << ", " << point.z << std::endl;
        // }
        

        // Find points that cluster the vertical plane
        pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
        tree->setInputCloud(vertical_plane);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
        ec.setClusterTolerance(1.0); // 0.5 meters
        ec.setMinClusterSize(20);
        ec.setMaxClusterSize(25000);
        ec.setSearchMethod(tree);
        ec.setInputCloud(vertical_plane);
        ec.extract(cluster_indices);


        for (const auto& cluster : cluster_indices)
        {
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZI>);
            for (const auto& idx : cluster.indices)
            {
                cloud_cluster->push_back((*vertical_plane)[idx]);
            }
            cloud_cluster->width = cloud_cluster->size();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;


            // Compute and save the bounding box or min/max points
            pcl::PointXYZI min_pt, max_pt;
            pcl::getMinMax3D<pcl::PointXYZI>(*cloud_cluster, min_pt, max_pt);

            // Create a BoundingBox message
            sar_msgs::BoundingBox bbox;
            bbox.min_point.x = min_pt.x;
            bbox.min_point.y = min_pt.y;
            bbox.min_point.z = min_pt.z;
            bbox.max_point.x = max_pt.x;
            bbox.max_point.y = max_pt.y;
            bbox.max_point.z = max_pt.z;

            bbox_array.boxes.push_back(bbox);

            std::cout << "Cluster " << i << " has " << cloud_cluster->size() << " points" << std::endl;
            std::cout << std::setprecision(3) << "Bounding Box: " << min_pt.x << ", " << min_pt.y << ", " << min_pt.z << " to " << max_pt.x << ", " << max_pt.y << ", " << max_pt.z << std::endl;

        }

        
        // Remove the inliers from the point cloud
        extract.setNegative(true); // Extract the outliers
        extract.filter(*cloud); // Update the point cloud without the vertical plane

        i++;
    }
       
    // // Publish the BoundingBoxArray message
    bbox_array.header = msg->header;
    pub_bounding_boxes.publish(bbox_array);


    // Convert to ROS data type and publish
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*vertical_planes_accumulated, output);
    output.header.frame_id = msg->header.frame_id;
    output.header.stamp = ros::Time::now();

    pub_processed_cloud.publish(output);

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_processor");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/SAR_Internal/lidar/raw", 1, cloud_cb, ros::TransportHints().tcpNoDelay());

    pcl::console::setVerbosityLevel(pcl::console::L_ERROR);

    pub_processed_cloud = nh.advertise<sensor_msgs::PointCloud2>("SAR_Internal/lidar/processed_cloud", 1);
    pub_bounding_boxes = nh.advertise<sar_msgs::BoundingBoxArray>("SAR_Internal/lidar/bounding_boxes_raw", 1);
    std::cout << "Node Running" << std::endl;
    ros::spin();




  return 0;
}
