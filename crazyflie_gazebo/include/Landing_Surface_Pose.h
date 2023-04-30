#include <iostream>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/common/common.hh>



#include <ros/ros.h>
#include "gazebo_msgs/SetModelState.h"





namespace gazebo {

    class Landing_Surface_Pose: public ModelPlugin
    {
        public:
            
        protected:
            void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
            void Init();
            void OnUpdate();
            bool Service_Callback(gazebo_msgs::SetModelState::Request &req, gazebo_msgs::SetModelState::Response &res);


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

            // POSE VALUES
            ignition::math::Vector3d Pos_0;     // [m]
            ignition::math::Quaterniond Quat_0;    
            ignition::math::Pose3d Pose_0;

    };

}
