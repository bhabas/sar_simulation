#include <iostream>
#include <thread>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/common/common.hh>
#include <random>



#include <ros/ros.h>
#include "crazyflie_msgs/OF_SensorData.h"


namespace gazebo {

    class OF_SensorPlugin: public ModelPlugin
    {
        public:
            
        protected:
            void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
            void OnUpdate();
            void Publish_OF_Data();
            float GaussianKernel(double mu, double sigma);


        private:
            physics::ModelPtr model_;
            physics::LinkPtr link_ptr;


            event::ConnectionPtr updateConnection;
            // DEFINE THREAD OBJECTS
            std::thread publisherThread;

            std::string linkName;
            std::string topicName;

            int updateRate;

            float _H_CEILING;

            float Vx_rel = 0.0; // [m/s]
            float Vy_rel = 0.0; // [m/s]
            float Vz_rel = 0.0; // [m/s]
            float d_ceil = 0.0; // [m]

            float Tau = 0.0;   // [s]
            float RREV = 0.0;  // [rad/s]
            float OFx = 0.0;   // [rad/s]
            float OFy = 0.0;   // [rad/s]

            float Tau_gaussianNoise;
            float OFx_gaussianNoise;
            float OFy_gaussianNoise;
            float RREV_gaussianNoise;

            ros::NodeHandle nh;
            ros::Publisher OF_Publisher;
            crazyflie_msgs::OF_SensorData OF_Data_msg;
            // ros::Subscriber MS_Subscriber = nh.subscribe<crazyflie_msgs::MS>("/MS", 1, &OF_SensorPlugin::MotorSpeedCallback, this, ros::TransportHints().tcpNoDelay());
    };

}

