#include <iostream>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/common/common.hh>


#include <ros/ros.h>
#include "sar_msgs/domainRand.h"


namespace gazebo {

    class Domain_Randomization_Plugin: public ModelPlugin
    {
        public:
            
        protected:
            void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
            bool UpdateInertia(sar_msgs::domainRand::Request &req, sar_msgs::domainRand::Response &res);



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
