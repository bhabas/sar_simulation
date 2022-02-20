#include "stabilizer_types.h"
#include "nml.h"

#include <iostream>
#include <thread>
#include <cmath>        // std::abs


#include "controller_gtc.h"
#include "estimator.h"
#include <ros/ros.h>

#include <stdio.h>
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "crazyflie_msgs/OF_SensorData.h"



uint32_t tick;

// State variables for the stabilizer
static setpoint_t setpoint;
static sensorData_t sensorData;
static state_t state;
static control_t control;

class StateCollector
{
    public:

        StateCollector(ros::NodeHandle *nh)
        {
            Vicon_Subscriber = nh->subscribe("/UKF/viconState_Filtered",1,&StateCollector::viconState_Callback,this,ros::TransportHints().tcpNoDelay());
            IMU_Subscriber = nh->subscribe("/CF_Internal/IMU",1,&StateCollector::imuState_Callback,this,ros::TransportHints().tcpNoDelay());
            OF_Subscriber = nh->subscribe("/CF_Internal/OF_Sensor",1,&StateCollector::OFState_Callback,this,ros::TransportHints().tcpNoDelay());

            controllerThread = std::thread(&StateCollector::stabilizerLoop, this);
        }

        void viconState_Callback(const nav_msgs::Odometry::ConstPtr &msg);
        void imuState_Callback(const sensor_msgs::Imu::ConstPtr &msg);
        void OFState_Callback(const crazyflie_msgs::OF_SensorData::ConstPtr &msg);
        void stabilizerLoop();

        ros::Subscriber Vicon_Subscriber;
        ros::Subscriber IMU_Subscriber;
        ros::Subscriber OF_Subscriber;

        // DEFINE THREAD OBJECTS
        std::thread controllerThread;

        uint32_t tick;


};

void StateCollector::OFState_Callback(const crazyflie_msgs::OF_SensorData::ConstPtr &msg)
{
    sensorData.Tau = msg->Tau;
    sensorData.OFx = msg->OFx;
    sensorData.OFy = msg->OFy;
    sensorData.RREV = msg->RREV;

}

void StateCollector::imuState_Callback(const sensor_msgs::Imu::ConstPtr &msg)
{
    sensorData.acc.x = msg->linear_acceleration.x/9.8066; // Convert to Gs to match crazyflie sensors
    sensorData.acc.y = msg->linear_acceleration.y/9.8066;
    sensorData.acc.z = msg->linear_acceleration.z/9.8066;

}

void StateCollector::viconState_Callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    
    // UPDATE POSE FROM VICON SYSTEM
    state.position.x = msg->pose.pose.position.x;
    state.position.y = msg->pose.pose.position.y;
    state.position.z = msg->pose.pose.position.z;

    state.attitudeQuaternion.x = msg->pose.pose.orientation.x;
    state.attitudeQuaternion.y = msg->pose.pose.orientation.y;
    state.attitudeQuaternion.z = msg->pose.pose.orientation.z;
    state.attitudeQuaternion.w = msg->pose.pose.orientation.w;

    // UPDATE VELOCITIES FROM VICON SYSTEM
    state.velocity.x = msg->twist.twist.linear.x;
    state.velocity.y = msg->twist.twist.linear.y;
    state.velocity.z = msg->twist.twist.linear.z;

    sensorData.gyro.x = msg->twist.twist.angular.x*180.0/M_PI; // Convert to deg/s to match crazyflie sensors
    sensorData.gyro.y = msg->twist.twist.angular.y*180.0/M_PI;
    sensorData.gyro.z = msg->twist.twist.angular.z*180.0/M_PI;

}


