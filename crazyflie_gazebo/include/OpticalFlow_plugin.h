/*
This script reads the current quadrotor position relative to the generated plane and returns
the absolute optical flow values based off of velocity and relative position. 

Derivation of equations can be found in Research_Notes_Book_2.pdf (December 2022)
*/

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
            ignition::math::Vector3d t_y;   // Plane Unit Tangent Vector


            float D_perp = 0.0;     // Distance to plane [m]
            float V_perp = 0.0;     // Perp velocity [m/s]
            float V_tx = 0.0;       // Tangent_x velocity [m/s]
            float V_ty = 0.0;       // Tangent_y velocity [m/s]

            float Theta_x = 0.0;    // Optical Flow (x-vel) [rad/s]
            float Theta_y = 0.0;    // Optical Flow (y-vel) [rad/s]
            float Theta_z = 0.0;    // Optical Flow (z-vel) [rad/s]
            float Tau = 0.0;        // Time-to-Contact [s]


            float Theta_x_gaussianNoise;
            float Theta_y_gaussianNoise;
            float Theta_z_gaussianNoise;


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

