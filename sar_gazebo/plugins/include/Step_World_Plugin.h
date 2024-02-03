
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

namespace gazebo {

    class Step_World_Plugin: public WorldPlugin
    {
        public:
            
        protected:
            void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf);
            // bool Step_Sim(sar_msgs::Hinge_Params::Request &req, sar_msgs::Hinge_Params::Response &res);

        private:

            ros::NodeHandle nh;
            // ros::ServiceServer Hinge_Update_Service = nh.advertiseService("/SAR_Internal/Hinge_Joint_Update", &Step_World_Plugin::Step_Sim, this);


    };

}
