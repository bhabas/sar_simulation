
// C++ LIBRARIES
#include <iostream>
#include <thread>
#include <cmath>        // std::abs

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


// STANDARD LIBRARIES
#include <math.h>       
#include <stdio.h>
#include <stdint.h>

// CF LIBRARIES
#include "math3d.h"
#include "stabilizer_types.h"
#include "nml.h"

void stateEstimator(state_t *state);
void commanderGetSetpoint();

using namespace std;


#define PWM_MAX 60000
#define f_MAX (16.5)
#define g2Newton (9.81f/1000.0f)
#define Newton2g (1000.0f/9.81f)

// FUNCTION PRIMITIVES
void controllerGTCInit(void);
bool controllerGTCTest(void);
void controllerGTCReset(void);
void controllerGTCTraj(void);
void controllerGTC(state_t *state, const uint32_t tick);
void GTC_Command();





class Controller
{
    public:
        // CONSTRUCTOR TO START PUBLISHERS AND SUBSCRIBERS (Similar to Python's __init__() )
        Controller(ros::NodeHandle *nh){

            globalState_Subscriber = nh->subscribe("/env/vicon_state",1,&Controller::vicon_stateCallback,this,ros::TransportHints().tcpNoDelay());

            // state.position.x = 1.0;
            controllerThread = std::thread(&Controller::startController, this);
            

        }

        // DEFINE FUNCTION PROTOTYPES
        void startController();
        void vicon_stateCallback(const nav_msgs::Odometry::ConstPtr &msg);

        

    private:

        // SENSORS
        ros::Subscriber globalState_Subscriber;


        


        // INITIALIZE ROS MSG VARIABLES
        geometry_msgs::Point _position; 
        geometry_msgs::Vector3 _velocity;
        geometry_msgs::Quaternion _quaternion;
        geometry_msgs::Vector3 _omega;
        geometry_msgs::Vector3 _accel;
        float _t;



        // DEFINE THREAD OBJECTS
        std::thread controllerThread;

        state_t state;
        
        uint32_t tick = 0;

        






};

void Controller::vicon_stateCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    // Follow msg names from message details - "rqt -s rqt_msg" 
    
    // SET STATE VALUES INTO CLASS STATE VARIABLES
    _t = msg->header.stamp.toSec();
    _position = msg->pose.pose.position; 
    _velocity = msg->twist.twist.linear;

    // cout << state.position.x << endl;
    state.position.x = _position.x;
    state.position.y = _position.y;
    state.position.z = _position.z;

    state.velocity.x = _velocity.x;
    state.velocity.y = _velocity.y;
    state.velocity.z = _velocity.z;
    // cout << "HELJ" << endl;

}



void Controller::startController()
{
    ros::Rate rate(500);
    
    while(ros::ok)
    {
        stateEstimator(&state);
        commanderGetSetpoint();
        controllerGTC(&state, tick);


        tick++;
        rate.sleep();
    }
}


void stateEstimator(state_t *state)
{
}
void commanderGetSetpoint()
{

}