// STANDARD INCLUDES
#include <iostream>

// ROS AND GAZEBO INCLUDES
#include <ros/ros.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/common/common.hh>

// CUSTOM INCLUDES
#include "sar_msgs/CTRL_Data.h"
#include "sar_msgs/MS.h"

#define g2Newton (9.81f/1000.0f)
#define Newton2g (1000.0f/9.81f)


namespace gazebo {

    class Motor_Plugin: public ModelPlugin
    {
        public:
            
        protected:
            void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
            void OnUpdate();
            void UpdateForcesAndMoments();
            void CtrlData_Callback(const sar_msgs::CTRL_Data::ConstPtr &msg);


        private:

            // CONFIG PARAMS
            std::string SAR_Type;
            std::string SAR_Config;
            std::string Cam_Config;

            // GAZEBO POINTERS
            physics::WorldPtr World_Ptr;
            physics::ModelPtr Model_Ptr;
            physics::JointPtr Joint_Ptr;
            physics::LinkPtr Link_Ptr;


            // MOTOR PARAMETERS
            std::string Motor_Joint_Name;
            std::string Prop_Link_Name;

            int Motor_Number;
            int Turn_Direction;

            double Thrust_Coeff = 0.0;    // Thrust Coeff [N/rad/s]
            double Torque_Coeff = 0.0;    // Torque Coeff [N*m/rad/s]
            double C_tf = 0.0;        // Torque-Thrust Coeff [N*m/N]

            // FIRST ORDER FILTER BEHAVIOR
            float Thrust_input = 0.0f;  // Desired Thrust [N]
            double Tau_up = 5.0;              // Motor Time Constant (Up) [s]
            double Tau_down = 5.0;            // Motor Time Constant (Down) [s]
            double Sampling_time = 0.0;
            double Prev_Sim_time = 0.0;
            double Prev_Thrust = 0.0;
            
            // CACULATED VALUES
            double Thrust = 0.0;              // Calculated Thrust [N]
            double Torque = 0.0;              // Calculated Torque [N*m]
            double Rot_Vel = 0.0;      // Rotational Velocity [rad/s]
            double Rot_Vel_Slowdown;    // Slowed-down Rotational Velocity [rad/s]

            // GAZEBO CONNECTIONS
            event::ConnectionPtr updateConnection;

            // ROS CONNECTIONS
            ros::NodeHandle nh;
            ros::Subscriber CTRL_Data_Sub = nh.subscribe<sar_msgs::CTRL_Data>("/CTRL/data", 1, &Motor_Plugin::CtrlData_Callback, this, ros::TransportHints().tcpNoDelay());
            ros::Publisher MS_Data_Pub = nh.advertise<sar_msgs::MS>("/SAR_Internal/MS",1);
            sar_msgs::MS Thrust_msg;
    };

}
