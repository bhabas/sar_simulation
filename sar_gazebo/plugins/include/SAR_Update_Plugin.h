#include <iostream>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/common/common.hh>
#include <gazebo/sensors/sensors.hh>


#include <ros/ros.h>
#include <thread>

#include "sar_msgs/Camera_Params.h"
#include "sar_msgs/Inertia_Params.h"
#include "sar_msgs/Hinge_Params.h"


#define Deg2Rad M_PI/180
#define Y_AXIS 0    // First index on leg joint
#define Z_AXIS 1    // Second index on leg joint


namespace gazebo {

    class SAR_Update_Plugin: public ModelPlugin
    {
        public:
            
        protected:
            void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
            void Update_Camera();
            void Update_Inertia();
            void Update_Hinge();

            bool Update_Camera_Service(sar_msgs::Camera_Params::Request &req, sar_msgs::Camera_Params::Response &res);
            bool Update_Inertia_Service(sar_msgs::Inertia_Params::Request &req, sar_msgs::Inertia_Params::Response &res);
            bool Update_Hinge_Service(sar_msgs::Hinge_Params::Request &req, sar_msgs::Hinge_Params::Response &res);


        private:

            // CONFIG PARAMS
            std::string SAR_Type;
            std::string SAR_Config;
            std::string Cam_Config;

            // GAZEBO POINTERS
            physics::ModelPtr Config_Model_Ptr;
            physics::LinkPtr SAR_Body_Ptr; // Pointer to Base_Model
            physics::WorldPtr World_Ptr;



            // CAMERA POINTERS
            sensors::SensorPtr Sensor_Ptr;
            sensors::CameraSensor* Camera_Ptr;
            physics::LinkPtr Camera_Link_Ptr;
            std::string Camera_Joint_Name;

            // CAMERA CONFIG PARAMS
            double X_Offset;    // [m]
            double Y_Offset;    // [m]
            double Z_Offset;    // [m]
            double Pitch_Angle; // [deg]
            double FPS;         // [1/s]




            // INERTIA PARAMETERS
            double Ixx_Body;
            double Iyy_Body;
            double Izz_Body;
            double Mass_Body;

            // INERTIA POINTERS
            physics::InertialPtr Inertial_Ptr;




            // HINGE JOINT PARAMETERS
            double Ixx_Leg;     // Leg Link x-axis inertia [kg*m^2]
            double Iyy_Leg;     // Leg Link y-axis inertia [kg*m^2]
            double Izz_Leg;     // Leg Link z-axis inertia [kg*m^2]
            double Leg_Angle;   // Leg angle defined by model [deg]

            double K_Pitch;     // Stiffness about pitch axis [N*m/rad]
            double C_Pitch;     // Damping coefficient about pitch axis [N*m*s/rad]
            double DR_Pitch;    // Damping Ratio about pitch axis

            double K_Yaw;       // Stiffness about pitch axis [N*m/rad]
            double DR_Yaw;      // Damping coefficient about pitch axis [N*m*s/rad]
            double C_Yaw;       // Damping Ratio about pitch axis


            // LEG AND HINGE JOINT POINTERS
            physics::LinkPtr Leg_1_LinkPtr;
            physics::LinkPtr Leg_2_LinkPtr;
            physics::LinkPtr Leg_3_LinkPtr;
            physics::LinkPtr Leg_4_LinkPtr;

            physics::JointPtr Hinge_1_JointPtr;
            physics::JointPtr Hinge_2_JointPtr;
            physics::JointPtr Hinge_3_JointPtr;
            physics::JointPtr Hinge_4_JointPtr;


            // ROS SERVICES
            ros::NodeHandle nh;
            ros::ServiceServer Cam_Update_Service = nh.advertiseService("/SAR_Internal/Camera_Update", &SAR_Update_Plugin::Update_Camera_Service, this);
            ros::ServiceServer Inertia_Update_Service = nh.advertiseService("/SAR_Internal/Inertia_Update", &SAR_Update_Plugin::Update_Inertia_Service, this);
            ros::ServiceServer Hinge_Update_Service = nh.advertiseService("/SAR_Internal/Hinge_Joint_Update", &SAR_Update_Plugin::Update_Hinge_Service, this);

    };

}
