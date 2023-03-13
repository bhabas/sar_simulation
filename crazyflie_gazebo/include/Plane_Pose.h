#include <iostream>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/common/common.hh>



#include <ros/ros.h>
#include "crazyflie_msgs/ModelMove.h"




namespace gazebo {

    class Plane_Pose: public WorldPlugin
    {
        public:
            
        protected:
            void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf);
            void OnUpdate();
            bool Service_Callback(crazyflie_msgs::ModelMove::Request &req, crazyflie_msgs::ModelMove::Response &res);


        private:
            physics::WorldPtr World_Ptr;
            physics::ModelPtr Surface_Model_Ptr;
            physics::ModelPtr Origin_Model_Ptr;
            event::ConnectionPtr updateConnection;

            // ROS VALUES
            ros::NodeHandle nh;
            ros::ServiceServer CMD_Service;

            // INITIAL POSE VALUES
            ignition::math::Vector3d Pos_0;     // [m]
            ignition::math::Vector3d Eul_0;     // [deg]
            ignition::math::Pose3d Pose_0;

            // INITIAL VELOCITY PARAMETERS
            ignition::math::Vector3d Vel_0;     // [m/s]
            ignition::math::Vector3d Accel_0;   // [m/s^2]

            // CURRENT VEL AND ANG VEL CONDITIONS
            ignition::math::Vector3d Vel;       // [m/s]
            ignition::math::Vector3d Ang_Vel;   // [rad/s]

    };

}
