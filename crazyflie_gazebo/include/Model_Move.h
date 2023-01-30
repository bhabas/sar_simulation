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

            ignition::math::Vector3d Pos;
            ignition::math::Vector3d Vel;


            ignition::math::Vector3d Eul_0;
            ignition::math::Vector3d Eul_Lim;
            ignition::math::Vector3d Freq;


            ignition::math::Pose3d Pose;



            std::string linkName;

            event::ConnectionPtr updateConnection;
            ros::ServiceServer CMD_Service;

            double t_0;
            double t_delta; 


            ros::NodeHandle nh;
    };

}
