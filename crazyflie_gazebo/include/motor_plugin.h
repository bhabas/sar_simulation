#include <iostream>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/common/common.hh>




#include <ros/ros.h>
#include "crazyflie_msgs/RLCmd.h"
#include "crazyflie_msgs/MS.h"
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
            double rot_vel;
            uint16_t rotorPWM = 0;

            double thrust_coeff;
            double torque_coeff;

            double thrust;
            double torque;

            event::ConnectionPtr updateConnection;

            ros::NodeHandle nh;
            ros::Subscriber MS_Subscriber = nh.subscribe<crazyflie_msgs::MS>("/MS", 1, &GazeboMotorPlugin::MotorSpeedCallback, this, ros::TransportHints().tcpNoDelay());
    };

}


// Converts thrust in PWM to their respective values in grams
static inline float PWM2thrust(uint16_t M_PWM) 
{
    // Conversion values calculated from PWM to Thrust Curve
    float a = 3.31e4;
    float b = 1.12e1;
    float c = 8.72;
    float d = 3.26e4;

    float f = b*atan2f((float)M_PWM-d,a)+c;

    if(f<0)
    {
      f = 0;
    }

    return f;
}