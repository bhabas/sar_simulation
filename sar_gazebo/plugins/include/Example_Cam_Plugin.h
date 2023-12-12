
// STANDARD INCLUDES
#include <iostream>
#include <thread>


// ROS AND GAZEBO INCLUDES
#include <ros/ros.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/common/common.hh>
#include <gazebo/sensors/sensors.hh>

#include <sensor_msgs/Image.h>

// rosrun image_view image_view image:=/SAR_Internal/camera/image_raw

namespace gazebo {

    class Example_Cam_Plugin: public ModelPlugin
    {
        public:
            
        protected:
            void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
            void CamData_Callback(const sensor_msgs::Image::ConstPtr &Camera_msg);

            

        private:

            sensor_msgs::Image inverted_image_msg;

            ros::NodeHandle nh;
            ros::Subscriber Cam_Data_Sub = nh.subscribe<sensor_msgs::Image>("/SAR_Internal/camera/image_raw", 1, &Example_Cam_Plugin::CamData_Callback, this, ros::TransportHints().tcpNoDelay());
            ros::Publisher Cam_Data_Pub = nh.advertise<sensor_msgs::Image>("/SAR_Internal/camera/inverted_image", 5);
    };

}
