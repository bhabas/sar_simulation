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
            physics::WorldPtr world;
            physics::ModelPtr model_ptr;
            physics::LinkPtr link_ptr;


            ignition::math::Vector3d torque_vec;

            std::string linkName;

            event::ConnectionPtr updateConnection;


            bool executeMoment = false;

            ros::NodeHandle nh;
            ros::Subscriber MS_Subscriber = nh.subscribe<crazyflie_msgs::RLCmd>("/RL/cmd", 5, &ModelMoment::RLCmd_Callback, this, ros::TransportHints().tcpNoDelay());
    };

}
