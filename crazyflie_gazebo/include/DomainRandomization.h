#include <iostream>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/common/common.hh>


#include <ros/ros.h>
#include "crazyflie_msgs/domainRand.h"


namespace gazebo {

    class DomainRand_plugin: public ModelPlugin
    {
        public:
            
        protected:
            void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
            bool UpdateInertia(crazyflie_msgs::domainRand::Request &req, crazyflie_msgs::domainRand::Response &res);



        private:
            physics::WorldPtr world;
            physics::ModelPtr model_ptr;
            physics::LinkPtr link_ptr;

            physics::InertialPtr inertia_ptr;
            ros::ServiceServer DomainRandService;

            ros::NodeHandle nh;
            std::string linkName;




    };

}
