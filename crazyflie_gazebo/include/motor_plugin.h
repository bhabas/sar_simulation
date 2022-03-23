#include <iostream>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/common/common.hh>




#include <ros/ros.h>
#include "crazyflie_msgs/RLCmd.h"
#include "crazyflie_msgs/CtrlData.h"
#define g2Newton (9.81f/1000.0f)
#define Newton2g (1000.0f/9.81f)


namespace gazebo {

    class GazeboMotorPlugin: public ModelPlugin
    {
        public:
            
        protected:
            void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
            void OnUpdate();
            void updateThrust();
            void updateTorque();
            void CtrlData_Callback(const crazyflie_msgs::CtrlData::ConstPtr &msg);


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
            double rot_vel;
            float MotorThrust = 0.0f;

            double thrust_coeff;
            double torque_coeff;

            double thrust;
            double torque;

            event::ConnectionPtr updateConnection;

            ros::NodeHandle nh;
            ros::Subscriber CTRL_Data_Sub = nh.subscribe<crazyflie_msgs::CtrlData>("/CTRL/data", 1, &GazeboMotorPlugin::CtrlData_Callback, this, ros::TransportHints().tcpNoDelay());
    };

}
