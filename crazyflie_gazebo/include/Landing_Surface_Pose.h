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

    class Landing_Surface_Pose: public ModelPlugin
    {
        public:
            
        protected:
            void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
            void Init();
            void OnUpdate();
            bool Service_Callback(gazebo_msgs::SetModelState::Request &req, gazebo_msgs::SetModelState::Response &res);
            void Wrench_Publisher();


        private:

            // GAZEBO POINTERS
            physics::WorldPtr World_Ptr;
            physics::ModelPtr Surface_Model_Ptr;
            physics::ModelPtr Origin_Model_Ptr;
            physics::JointPtr Joint_Ptr;

            std::string Joint_Name;

            bool UpdatingJoint = false;

            event::ConnectionPtr updateConnection;

            // ROS VALUES
            ros::NodeHandle nh;
            ros::ServiceServer CMD_Service;
            ros::Publisher Wrench_Pub;
            geometry_msgs::WrenchStamped WrenchData_msg;
            std::thread Wrench_Thread;

            physics::JointWrench wrench;
            ignition::math::Vector3d torque;
            ignition::math::Vector3d force;


            // POSE VALUES
            ignition::math::Vector3d Pos_0;     // [m]
            ignition::math::Quaterniond Quat_0;    
            ignition::math::Pose3d Pose_0;

    };

}
