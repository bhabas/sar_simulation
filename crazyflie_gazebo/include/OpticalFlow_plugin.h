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

    class OpticalFlow_plugin: public ModelPlugin
    {
        public:
            OpticalFlow_plugin();
            ~OpticalFlow_plugin();

            
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


            // PLANE CONFIG
            std::string Plane_Config;
            float Plane_Angle = 0.0;    // [rad]


            ignition::math::Vector3d r_PO;  // Plane Position Vector        [m]
            ignition::math::Vector3d r_BO;  // Quad Position Vector         [m]
            ignition::math::Vector3d r_PB;  // Quad-Plane Distance Vector   [m]
            ignition::math::Vector3d V_BO;  // Quad Velocity Vector         [m/s]


            ignition::math::Vector3d n_hat; // Plane Unit Normal Vector
            ignition::math::Vector3d t_x;   // Plane Unit Tangent Vector

            float D_perp = 0.0; // [m]
            float V_perp = 0.0; // [m/s]
            float V_tx = 0.0;   // [m/s]

            float Tau = 0.0;   // [s]
            float OFx = 0.0;   // [rad/s]

            float Tau_gaussianNoise;
            float OFx_gaussianNoise;

            // INIT ROS OBJECTS
            ros::NodeHandle nh;
            ros::Publisher OF_Publisher;
            crazyflie_msgs::OF_SensorData OF_Data_msg;
    };

    OpticalFlow_plugin::OpticalFlow_plugin()
    {

    };

    OpticalFlow_plugin::~OpticalFlow_plugin()
    {
        publisherThread.join();
    };

}

