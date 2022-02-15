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
            void OnUpdate(const common::UpdateInfo&  /*_info*/);
            void MotorSpeedCallback(const crazyflie_msgs::MS::ConstPtr &msg);


        private:
            physics::WorldPtr world_;
            physics::ModelPtr model_;

            ros::NodeHandle nh;
            ros::Subscriber MS_Subscriber = nh.subscribe<crazyflie_msgs::MS>("/MS", 1, &GazeboMotorPlugin::MotorSpeedCallback, this, ros::TransportHints().tcpNoDelay());
    };

}