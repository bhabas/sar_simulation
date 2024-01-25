#include <iostream>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/common/common.hh>



#include <ros/ros.h>
#include <thread>
#include "gazebo_msgs/SetModelState.h"
#include "geometry_msgs/WrenchStamped.h"
#include "sar_msgs/Surface_Settings.h"




#define Deg2Rad M_PI/180
#define Rad2Deg 180/M_PI

namespace gazebo {

    class Landing_Surface_Pose: public ModelPlugin
    {
        public:
            
        protected:
            void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
            void Update_Pose();
            void OnUpdate();
            bool Service_Callback(sar_msgs::Surface_Settings::Request &req, sar_msgs::Surface_Settings::Response &res);


        private:

            // SDF PARAMS
            std::string Joint_Name;
            std::string ForceTorque_Topic;


            // GAZEBO POINTERS
            physics::WorldPtr World_Ptr;
            physics::ModelPtr World_Origin_Model_Ptr;
            physics::LinkPtr  World_Origin_Link_Ptr;

            physics::ModelPtr Surface_Model_Ptr;
            physics::LinkPtr  Surface_Link_Ptr;
            physics::JointPtr Joint_Ptr;

            event::ConnectionPtr updateConnection;


            // POSE UPDATES
            ros::NodeHandle nh;
            ros::ServiceServer Pose_Update_Service;
            ignition::math::Vector3d Pos;
            ignition::math::Quaterniond Quat;    
            ignition::math::Pose3d Pose;

            bool UpdatingJoint = false;

            // POSE CONFIG SETTINGS
            double Pos_X;    // [m]
            double Pos_Y;    // [m]
            double Pos_Z;    // [m]
            double Plane_Angle_deg; // [deg]

            // FORCE TORQUE UPDATES
            ros::Publisher ForceTorque_Pub;
            geometry_msgs::WrenchStamped ForceTorque_msg;
            
    };

}
