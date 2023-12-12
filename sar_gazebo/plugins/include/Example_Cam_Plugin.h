
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

    class Example_Cam_Plugin: public ModelPlugin
    {
        public:
            
        protected:
            void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

        private:

            ros::NodeHandle nh;

    };

}
