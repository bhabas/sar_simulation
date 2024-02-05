
// STANDARD INCLUDES
#include <iostream>
#include <thread>


// ROS AND GAZEBO INCLUDES
#include <ros/ros.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/common/common.hh>
#include <gazebo/sensors/sensors.hh>

// CUSTOM INCLUDE
#include "sar_msgs/World_Step.h"


namespace gazebo {

    class Step_World_Plugin: public WorldPlugin
    {
        public:
            
        protected:
            void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf);
            bool Step_World(sar_msgs::World_Step::Request &req, sar_msgs::World_Step::Response &res);

        private:

            physics::WorldPtr World_Ptr;

            
            ros::NodeHandle nh;
            ros::ServiceServer Hinge_Update_Service = nh.advertiseService("/ENV/World_Step", &Step_World_Plugin::Step_World, this);


    };

}
