#include <iostream>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/common/common.hh>



#include <ros/ros.h>
#include "crazyflie_msgs/ModelMove.h"




namespace gazebo {

    class Plane_Pose: public ModelPlugin
    {
        public:
            
        protected:
            void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
            void OnUpdate();
            bool Service_Callback(crazyflie_msgs::ModelMove::Request &req, crazyflie_msgs::ModelMove::Response &res);


        private:
            physics::WorldPtr world_ptr;
            physics::ModelPtr model_ptr;
            physics::JointPtr joint_ptr;
            event::ConnectionPtr updateConnection;

            // ROS VALUES
            ros::NodeHandle nh;
            ros::ServiceServer CMD_Service;

            // TIME VALUES
            double t_0;     // Starting Time [s]
            double t_delta; // Time since start [s]

            // INITIAL POSE VALUES
            ignition::math::Vector3d Pos_0;     // [m]
            ignition::math::Vector3d Eul_0;     // [deg]
            ignition::math::Pose3d Pose_0;

            // INITIAL VELOCITY PARAMETERS
            ignition::math::Vector3d Vel_0;     // [m/s]
            ignition::math::Vector3d Accel_0;   // [m/s^2]

            // OSCILLATION PARAMETERS
            ignition::math::Vector3d Eul_Lim;   // [deg]
            ignition::math::Vector3d Freq;      // [Hz]

            // CURRENT VEL AND ANG VEL CONDITIONS
            ignition::math::Vector3d Vel;       // [m/s]
            ignition::math::Vector3d Ang_Vel;   // [rad/s]

    };

}
