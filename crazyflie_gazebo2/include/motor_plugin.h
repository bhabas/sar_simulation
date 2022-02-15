#include <iostream>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/common/common.hh>




#include <ros/ros.h>
#include "crazyflie_msgs/RLCmd.h"
#include "crazyflie_msgs/MS.h"

namespace gazebo {

    class GazeboMotorPlugin: public ModelPlugin
    {
        public:
            
        protected:
            void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
            void OnUpdate();
            void updateThrust();
            void updateTorque();
            void MotorSpeedCallback(const crazyflie_msgs::MS::ConstPtr &msg);


        private:
            physics::WorldPtr world_;
            physics::ModelPtr model_;
            physics::JointPtr joint_ptr;
            physics::LinkPtr link_ptr;

            int motor_number;
            int turning_direction;

            std::string jointName;
            std::string linkName;

            double rot_vel_visual_slowdown;
            double rot_vel = 0;

            double thrust_coeff;
            double torque_coeff;

            double thrust;
            double torque;

            event::ConnectionPtr updateConnection;

            ros::NodeHandle nh;
            ros::Subscriber MS_Subscriber = nh.subscribe<crazyflie_msgs::MS>("/MS", 1, &GazeboMotorPlugin::MotorSpeedCallback, this, ros::TransportHints().tcpNoDelay());
    };

}