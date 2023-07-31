#include <iostream>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/common/common.hh>



#include <ros/ros.h>
#include <thread>
#include "gazebo_msgs/SetModelState.h"
#include "geometry_msgs/WrenchStamped.h"





namespace gazebo {

    class Cam_Update_Plugin: public ModelPlugin
    {
        public:
            
        protected:
            void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
            void OnUpdate();
            bool Service_Callback(gazebo_msgs::SetModelState::Request &req, gazebo_msgs::SetModelState::Response &res);
            void ForceTorque_Publisher();


        private:

            // SDF PARAMS
            std::string Joint_Name;
            std::string ForceTorque_Topic;


            // GAZEBO POINTERS
            physics::ModelPtr Base_Model_Ptr;
            physics::LinkPtr Camera_Link_Ptr;
            physics::LinkPtr SAR_Body_Ptr;

            physics::JointPtr Joint_Ptr;

            event::ConnectionPtr updateConnection;


            // POSE UPDATES
            ros::NodeHandle nh;
            ros::ServiceServer Pose_Update_Service;
            ignition::math::Vector3d Pos;
            ignition::math::Quaterniond Quat;    
            ignition::math::Pose3d Pose;

            bool UpdatingJoint = false;

            // ROS VALUES

            // FORCE TORQUE UPDATES
            std::thread ForceTorque_Publisher_Thread;
            ros::Publisher ForceTorque_Pub;
            geometry_msgs::WrenchStamped ForceTorque_msg;
            

            ignition::math::Vector3d Force_Vec;
            ignition::math::Vector3d Torque_Vec;

            
    };

}
