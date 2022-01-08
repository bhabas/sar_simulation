#include <iostream>
#include <thread>
#include <cmath>        // std::abs
#include <math.h>       
#include "math3d.h"
#include "nml.h"

#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <math.h>
#include <unistd.h>


// ROS Includes
#include <ros/ros.h>
#include "crazyflie_msgs/CtrlData.h"
#include "crazyflie_msgs/ImpactData.h"
#include "crazyflie_msgs/RLCmd.h"
#include "crazyflie_msgs/RLData.h"
#include "crazyflie_msgs/PadConnect.h"
#include "crazyflie_msgs/Policy_Values.h"
#include "crazyflie_msgs/MS.h"


#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "gazebo_msgs/SetPhysicsProperties.h"



using namespace std;

#define PWM_MAX 60000
#define f_MAX (16.5)
#define g2Newton (9.81f/1000.0f)
#define Newton2g (1000.0f/9.81f)




class Controller
{
    public:
        // CONSTRUCTOR TO START PUBLISHERS AND SUBSCRIBERS (Similar to Python's __init__() )
        Controller(ros::NodeHandle *nh){
            // ctrl_Publisher = nh->advertise<crazyflie_msgs::CtrlData>("/ctrl_data",1);
            // MS_Publisher = nh->advertise<crazyflie_msgs::MS>("/MS",1);

            // // NOTE: tcpNoDelay() removes delay where system is waiting for datapackets to be fully filled before sending;
            // // instead of sending data as soon as it is available to match publishing rate (This is an issue with large messages like Odom or Custom)
            // // Queue lengths are set to '1' so only the newest data is used


            // // BODY SENSORS
            // OF_Subscriber = nh->subscribe("/cf1/OF_sensor",1,&Controller::OFCallback,this,ros::TransportHints().tcpNoDelay()); 
            // imu_Subscriber = nh->subscribe("/cf1/imu",1,&Controller::imuCallback,this);
                
            // // ENVIRONMENT SENSORS
            // globalState_Subscriber = nh->subscribe("/env/vicon_state",1,&Controller::vicon_stateCallback,this,ros::TransportHints().tcpNoDelay());
            // ceilingFT_Subcriber = nh->subscribe("/env/ceiling_force_sensor",5,&Controller::ceilingFTCallback,this,ros::TransportHints().tcpNoDelay());


            // // COMMANDS AND INFO
            // RLCmd_Subscriber = nh->subscribe("/rl_ctrl",50,&Controller::GTC_Command,this);
            // RLData_Subscriber = nh->subscribe("/rl_data",5,&Controller::RLData_Callback,this,ros::TransportHints().tcpNoDelay());
            // SimSpeed_Client = nh->serviceClient<gazebo_msgs::SetPhysicsProperties>("/gazebo/set_physics_properties");
        

            

        }

        // DEFINE FUNCTION PROTOTYPES
        void Load();
        void controllerGTC();
        








    private:
        // DEFINE PUBLISHERS AND SUBSCRIBERS
        ros::Publisher ctrl_Publisher;
        ros::Publisher MS_Publisher;

        // SENSORS
        ros::Subscriber globalState_Subscriber;
        ros::Subscriber OF_Subscriber;
        ros::Subscriber imu_Subscriber;


        ros::Subscriber ceilingFT_Subcriber;


        // COMMANDS AND INFO
        ros::Subscriber RLCmd_Subscriber;
        ros::Subscriber RLData_Subscriber;
        ros::ServiceClient SimSpeed_Client;


        


        // INITIALIZE ROS MSG VARIABLES
        geometry_msgs::Point _position; 
        geometry_msgs::Vector3 _velocity;
        geometry_msgs::Quaternion _quaternion;
        geometry_msgs::Vector3 _omega;
        geometry_msgs::Vector3 _accel;



        // DEFINE THREAD OBJECTS
        std::thread controllerThread;


        


        






};


void Controller::Load()
{
    cout << setprecision(3);
    cout << fixed;





    


    // START COMMUNICATION THREADS
    controllerThread = std::thread(&Controller::controllerGTC, this);


}

