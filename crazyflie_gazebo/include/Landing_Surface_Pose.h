#include <iostream>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/common/common.hh>



#include <ros/ros.h>
#include "crazyflie_msgs/ModelMove.h"




namespace gazebo {

    class Landing_Surface_Pose: public WorldPlugin
    {
        public:
            
        protected:
            void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf);
            void OnUpdate();
            bool Service_Callback(crazyflie_msgs::ModelMove::Request &req, crazyflie_msgs::ModelMove::Response &res);


        private:

            // GAZEBO POINTERS
            physics::WorldPtr World_Ptr;
            physics::ModelPtr Surface_Model_Ptr;
            physics::ModelPtr Origin_Model_Ptr;
            physics::JointPtr Joint_Ptr;

            event::ConnectionPtr updateConnection;

            // ROS VALUES
            ros::NodeHandle nh;
            ros::ServiceServer CMD_Service;

            // POSE VALUES
            ignition::math::Vector3d Pos_0;     // [m]
            ignition::math::Vector3d Eul_0;     // [deg]
            ignition::math::Pose3d Pose_0;

    };

}
