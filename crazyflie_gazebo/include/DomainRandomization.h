#include <iostream>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/common/common.hh>




#include <ros/ros.h>
#include "crazyflie_msgs/RLCmd.h"
#include "crazyflie_msgs/activateSticky.h"
#include "crazyflie_msgs/domainRand.h"





namespace gazebo {

    class DomainRand_plugin: public ModelPlugin
    {
        public:
            
        protected:
            void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
            void RLCmd_Callback(const crazyflie_msgs::RLCmd::ConstPtr &msg);
            bool UpdateInertia(crazyflie_msgs::domainRand::Request &req, crazyflie_msgs::domainRand::Response &res);



        private:
            physics::WorldPtr world;
            physics::ModelPtr model_ptr;
            physics::LinkPtr link_ptr;

            physics::InertialPtr inertia_ptr;
            ros::ServiceServer DomainRandService;

            std::string linkName;




            ros::NodeHandle nh;
            ros::Subscriber MS_Subscriber = nh.subscribe<crazyflie_msgs::RLCmd>("/RL/cmd", 5, &DomainRand_plugin::RLCmd_Callback, this, ros::TransportHints().tcpNoDelay());
    };

}
