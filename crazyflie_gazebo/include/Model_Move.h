#include <iostream>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/common/common.hh>




#include <ros/ros.h>
#include "crazyflie_msgs/ModelMove.h"



namespace gazebo {

    class ModelMove: public ModelPlugin
    {
        public:
            
        protected:
            void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
            void OnUpdate();
            bool Service_Callback(crazyflie_msgs::ModelMove::Request &req, crazyflie_msgs::ModelMove::Response &res);


        private:
            physics::WorldPtr world;
            physics::ModelPtr model_ptr;
            physics::LinkPtr link_ptr;

            ignition::math::Vector3d vel_vec;


            std::string linkName;

            event::ConnectionPtr updateConnection;
            ros::ServiceServer CMD_Service;


            ros::NodeHandle nh;
    };

}
