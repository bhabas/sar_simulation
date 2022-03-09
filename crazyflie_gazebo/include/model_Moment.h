#include <iostream>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/common/common.hh>




#include <ros/ros.h>
#include "crazyflie_msgs/RLCmd.h"



namespace gazebo {

    class ModelMoment: public ModelPlugin
    {
        public:
            
        protected:
            void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
            void OnUpdate();
            void RLCmd_Callback(const crazyflie_msgs::RLCmd::ConstPtr &msg);


        private:
            physics::WorldPtr world_;
            physics::ModelPtr model_;


            event::ConnectionPtr updateConnection;

            ros::NodeHandle nh;
            ros::Subscriber MS_Subscriber = nh.subscribe<crazyflie_msgs::RLCmd>("/RL/cmd", 1, &ModelMoment::RLCmd_Callback, this, ros::TransportHints().tcpNoDelay());
    };

}
