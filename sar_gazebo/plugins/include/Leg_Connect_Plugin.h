#include <iostream>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/common/common.hh>
#include <gazebo/sensors/sensors.hh>



#include <ros/ros.h>
#include <thread>


namespace gazebo {

    class Leg_Connect_Plugin: public ModelPlugin
    {
        public:
            
        protected:
            void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
            void OnUpdate();

        private:

            // SDF PARAMS
            std::string Joint_Name;
            std::string Link_Name;

            bool Sticky_Flag = false;

            // CONFIG PARAMS
            std::string SAR_Type;
            std::string SAR_Config;
            std::string Cam_Config;
            std::string Collision_Name;

            // GAZEBO POINTERS
            physics::WorldPtr World_Ptr;
            physics::ModelPtr Model_Ptr;
            physics::JointPtr Contact_Joint_Ptr;

            double collision_radius = 0.01;

            ignition::math::Vector3d contactPositionWorld;  //
            ignition::math::Vector3d contactPositionLocal;  // With reference to contact surface link frame 
            ignition::math::Pose3d contactPose;
            

            physics::LinkPtr Leg_Link_Ptr;
            physics::LinkPtr Surface_Link_Ptr;




            event::ConnectionPtr updateConnection;



            // POSE UPDATES
            ros::NodeHandle nh;
            ros::ServiceServer Cam_Update_Service;
    };

}
