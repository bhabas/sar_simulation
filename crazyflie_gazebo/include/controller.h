
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

            viconState_Subscriber = nh->subscribe("/env/vicon_state",1,&Controller::vicon_Callback,this,ros::TransportHints().tcpNoDelay());
            imu_Subscriber = nh->subscribe("/cf1/imu",1,&Controller::imu_Callback,this,ros::TransportHints().tcpNoDelay());
            OF_Subscriber = nh->subscribe("/cf1/OF_sensor",1,&Controller::OF_Callback,this,ros::TransportHints().tcpNoDelay()); 

            // state.position.x = 1.0;
            controllerThread = std::thread(&Controller::startController, this);
            

        }

        // DEFINE FUNCTION PROTOTYPES
        void startController();
        void vicon_Callback(const nav_msgs::Odometry::ConstPtr &msg);
        void imu_Callback(const sensor_msgs::Imu::ConstPtr &msg);
        void OF_Callback(const nav_msgs::Odometry::ConstPtr &msg);           




        

    private:

        // SENSORS
        ros::Subscriber viconState_Subscriber;
        ros::Subscriber imu_Subscriber;
        ros::Subscriber OF_Subscriber;


        


        // INITIALIZE ROS MSG VARIABLES
        geometry_msgs::Point _position; 
        geometry_msgs::Vector3 _velocity;
        geometry_msgs::Quaternion _quaternion;
        geometry_msgs::Vector3 _omega;
        geometry_msgs::Vector3 _accel;
        float _t;

        float _H_CEILING = 2.10f;


        float _RREV;
        float _OF_x;
        float _OF_y;



        // DEFINE THREAD OBJECTS
        std::thread controllerThread;

        state_t state;
        sensorData_t sensors;
        
        uint32_t tick = 0;

        






};

void Controller::vicon_Callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    // Follow msg names from message details - "rqt -s rqt_msg" 
    
    // SET STATE VALUES INTO CLASS STATE VARIABLES
    _t = msg->header.stamp.toSec();
    _position = msg->pose.pose.position; 
    _velocity = msg->twist.twist.linear;

    state.position.x = _position.x;
    state.position.y = _position.y;
    state.position.z = _position.z;

    state.velocity.x = _velocity.x;
    state.velocity.y = _velocity.y;
    state.velocity.z = _velocity.z;

}

void Controller::imu_Callback(const sensor_msgs::Imu::ConstPtr &msg){
    _quaternion = msg->orientation;
    _omega = msg->angular_velocity;
    _accel = msg->linear_acceleration;

    state.attitudeQuaternion.x = _quaternion.x;
    state.attitudeQuaternion.y = _quaternion.y;
    state.attitudeQuaternion.z = _quaternion.z;
    state.attitudeQuaternion.w = _quaternion.w;

    sensors.gyro.x = _omega.x;
    sensors.gyro.y = _omega.y;
    sensors.gyro.z = _omega.z;

    sensors.acc.x = _accel.x;
    sensors.acc.y = _accel.y;
    sensors.acc.z = _accel.z;
    

}

void Controller::OF_Callback(const nav_msgs::Odometry::ConstPtr &msg){

    const geometry_msgs::Point position = msg->pose.pose.position; 
    const geometry_msgs::Vector3 velocity = msg->twist.twist.linear;

    
    double d = _H_CEILING-position.z; // h_ceiling - height

    // SET SENSOR VALUES INTO CLASS VARIABLES
    // _RREV = msg->RREV;
    // _OF_x = msg->OF_x;
    // _OF_y = msg->OF_y;

    _RREV = velocity.z/d;
    _OF_x = -velocity.y/d;
    _OF_y = -velocity.x/d;
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