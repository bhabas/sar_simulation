// STANDARD IMPORTS
#include <iostream>
#include <thread>
#include <random>
#include <math.h>
#include <boost/algorithm/clamp.hpp>

// GAZEBO IMPORTS
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/common/common.hh>

// ROS IMPORTS
#include <ros/ros.h>
#include "crazyflie_msgs/OF_SensorData.h"


namespace gazebo {

    class OF_SensorPlugin: public ModelPlugin
    {
        public:
            OF_SensorPlugin();
            ~OF_SensorPlugin();

            
        protected:
            void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
            void Publish_OF_Data();
            float GaussianKernel(double mu, double sigma);


        private:
            physics::ModelPtr model_ptr;
            physics::LinkPtr link_ptr;


            // DEFINE THREAD OBJECTS
            std::thread publisherThread;

            // CONSTANTS
            std::string linkName;
            std::string topicName;
            int updateRate;     // [hz]
            float _H_CEILING;   // [m]


            // INITIALIZE VARIABLES
            float Vx_rel = 0.0; // [m/s]
            float Vy_rel = 0.0; // [m/s]
            float Vz_rel = 0.0; // [m/s]
            float d_ceil = 0.0; // [m]

            float Tau = 0.0;   // [s]
            float OFx = 0.0;   // [rad/s]
            float OFy = 0.0;   // [rad/s]

            float Tau_gaussianNoise;
            float OFx_gaussianNoise;
            float OFy_gaussianNoise;

            // INIT ROS OBJECTS
            ros::NodeHandle nh;
            ros::Publisher OF_Publisher;
            crazyflie_msgs::OF_SensorData OF_Data_msg;
    };

    OF_SensorPlugin::OF_SensorPlugin()
    {
    };

    OF_SensorPlugin::~OF_SensorPlugin()
    {
        publisherThread.join();
    };

}

