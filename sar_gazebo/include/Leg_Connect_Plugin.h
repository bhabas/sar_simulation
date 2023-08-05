#include <iostream>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/common/common.hh>
#include <gazebo/sensors/sensors.hh>



#include <ros/ros.h>
#include <thread>
#include "sar_msgs/Cam_Settings.h"


namespace gazebo {

    class Leg_Connect_Plugin: public ModelPlugin
    {
        public:
            
        protected:
            void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
            void OnUpdate();
            bool Service_Callback(sar_msgs::Cam_Settings::Request &req, sar_msgs::Cam_Settings::Response &res);

        private:

            // SDF PARAMS
            std::string Joint_Name;

            // CONFIG PARAMS
            std::string SAR_Type;
            std::string SAR_Config;
            std::string Cam_Config;
            std::string Collision_Name;

            // GAZEBO POINTERS
            physics::WorldPtr World_Ptr;
            physics::ModelPtr Model_Ptr;
            physics::JointPtr Joint_Ptr;

            event::ConnectionPtr updateConnection;


            // POSE UPDATES
            ros::NodeHandle nh;
            ros::ServiceServer Cam_Update_Service;
    };

}
