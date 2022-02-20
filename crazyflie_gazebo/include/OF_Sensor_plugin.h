#include <iostream>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/common/common.hh>




#include <ros/ros.h>



namespace gazebo {

    class OF_SensorPlugin: public ModelPlugin
    {
        public:
            
        protected:
            void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
            void OnUpdate();


        private:
            physics::WorldPtr world_;
            physics::ModelPtr model_;
            // physics::JointPtr joint_ptr;
            physics::LinkPtr link_ptr;


            event::ConnectionPtr updateConnection;

            std::string linkName;

            double h_ceiling = 2.10;

            double Vx_rel = 0.0; // [m/s]
            double Vy_rel = 0.0; // [m/s]
            double Vz_rel = 0.0; // [m/s]
            double d_ceil = 0.0; // [m]

            double Tau = 0.0;   // [s]
            double RREV = 0.0;  // [rad/s]
            double OFx = 0.0;   // [rad/s]
            double OFy = 0.0;   // [rad/s]


            

            // ros::NodeHandle nh;
            // ros::Subscriber MS_Subscriber = nh.subscribe<crazyflie_msgs::MS>("/MS", 1, &OF_SensorPlugin::MotorSpeedCallback, this, ros::TransportHints().tcpNoDelay());
    };

}

