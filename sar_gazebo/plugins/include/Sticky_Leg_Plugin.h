#include <iostream>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/common/common.hh>
#include <gazebo/sensors/sensors.hh>

#include "sar_msgs/Sticky_Pad_Connect.h"
#include "sar_msgs/Activate_Sticky_Pads.h"

#include <ros/ros.h>
#include <thread>


namespace gazebo {

    class Sticky_Leg_Plugin: public ModelPlugin
    {
        public:
            
        protected:
            void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
            void OnUpdate();
            bool Service_Callback(sar_msgs::Activate_Sticky_Pads::Request &req, sar_msgs::Activate_Sticky_Pads::Response &res);


        private:

            // SDF PARAMS
            std::string Joint_Name;
            std::string Link_Name;
            int Leg_Number;

            bool Sticky_Flag = true;
            bool Attached_Flag = false;

            // CONFIG PARAMS
            std::string SAR_Type;
            std::string SAR_Config;
            std::string Cam_Config;
            std::string Collision_Name;

            // GAZEBO POINTERS
            physics::WorldPtr World_Ptr;
            physics::ModelPtr Model_Ptr;
            physics::JointPtr Contact_Joint_Ptr;

            double collision_radius = 0.002;

            ignition::math::Vector3d contactPositionWorld;  //
            ignition::math::Vector3d contactPositionLocal;  // With reference to contact surface link frame 
            ignition::math::Vector3d contactPositionWorldUpdated;
            ignition::math::Pose3d contactPose;
            

            physics::LinkPtr Leg_Link_Ptr;
            physics::LinkPtr Surface_Link_Ptr;


            sar_msgs::Sticky_Pad_Connect Sticky_Leg_Connect_msg;

            event::ConnectionPtr updateConnection;



            // POSE UPDATES
            ros::NodeHandle nh;
            ros::Publisher Sticky_Pad_Connect_Publisher = nh.advertise<sar_msgs::Sticky_Pad_Connect>("/SAR_Internal/Leg_Connections", 5);
            ros::ServiceServer Leg_Connect_Service;
    };

}
