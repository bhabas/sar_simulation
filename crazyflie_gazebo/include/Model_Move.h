#include <iostream>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/common/common.hh>




#include <ros/ros.h>
// #include "crazyflie_msgs/RLCmd.h"



namespace gazebo {

    class ModelMove: public ModelPlugin
    {
        public:
            
        protected:
            void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
            void OnUpdate();


        private:
            physics::WorldPtr world;
            physics::ModelPtr model_ptr;
            physics::LinkPtr link_ptr;

            physics::LinkPtr rotor1_ptr;
            physics::LinkPtr rotor4_ptr;
            

            ignition::math::Vector3d vel_vec;
            ignition::math::Vector3d thrust_vec;


            std::string linkName;

            event::ConnectionPtr updateConnection;


            bool executeMoment = false;

            ros::NodeHandle nh;
    };

}
