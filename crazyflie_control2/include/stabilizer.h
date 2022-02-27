
#include <iostream>
#include <thread>
#include <cmath>        // std::abs
#include <stdio.h>


#include "stabilizer_types.h"
#include "controller_gtc.h"
#include "estimator.h"
#include "nml.h"


#include <ros/ros.h>
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "crazyflie_msgs/OF_SensorData.h"
#include "crazyflie_msgs/MS.h"

#include "crazyflie_msgs/CtrlData.h"
#include "crazyflie_msgs/ImpactData.h"
#include "crazyflie_msgs/RLCmd.h"
#include "crazyflie_msgs/RLData.h"
#include "crazyflie_msgs/PadConnect.h"
#include "crazyflie_msgs/MS.h"


uint32_t tick;

// State variables for the stabilizer
static setpoint_t setpoint;
static sensorData_t sensorData;
static state_t state;
static control_t control;

class Controller
{
    public:

        Controller(ros::NodeHandle *nh)
        {
            // SUBSCRIBERS
            Vicon_Subscriber = nh->subscribe("/UKF/viconState_Filtered",1,&Controller::viconState_Callback,this,ros::TransportHints().tcpNoDelay());
            IMU_Subscriber = nh->subscribe("/CF_Internal/IMU",1,&Controller::imuState_Callback,this,ros::TransportHints().tcpNoDelay());
            OF_Subscriber = nh->subscribe("/CF_Internal/OF_Sensor",1,&Controller::OFState_Callback,this,ros::TransportHints().tcpNoDelay());
            CMD_Subscriber = nh->subscribe("/rl_ctrl",50,&Controller::CMD_Callback,this,ros::TransportHints().tcpNoDelay());
            CeilingFT_Subcriber = nh->subscribe("/env/ceiling_force_sensor",5,&Controller::ceilingFT_Callback,this,ros::TransportHints().tcpNoDelay());
            RLData_Subscriber = nh->subscribe("/rl_data",5,&Controller::RLData_Callback,this,ros::TransportHints().tcpNoDelay());

            // PUBLISHERS
            MS_PWM_Publisher = nh->advertise<crazyflie_msgs::MS>("/MS",1);

            controllerThread = std::thread(&Controller::stabilizerLoop, this);
        }

        

        // SUBSCRIBERS
        ros::Subscriber Vicon_Subscriber;
        ros::Subscriber IMU_Subscriber;
        ros::Subscriber OF_Subscriber;
        ros::Subscriber CMD_Subscriber;

        ros::Subscriber CeilingFT_Subcriber;
        ros::Subscriber RLData_Subscriber;

        // PUBLISHERS
        ros::Publisher MS_PWM_Publisher;

        // SERVICES
        ros::ServiceClient _SimSpeed_Client;

        // DEFINE THREAD OBJECTS
        std::thread controllerThread;

        uint32_t tick;

        // FUNCTION PRIMITIVES
        void viconState_Callback(const nav_msgs::Odometry::ConstPtr &msg);
        void imuState_Callback(const sensor_msgs::Imu::ConstPtr &msg);
        void OFState_Callback(const crazyflie_msgs::OF_SensorData::ConstPtr &msg);

        void CMD_Callback(const crazyflie_msgs::RLCmd::ConstPtr &msg);
        void ceilingFT_Callback(const crazyflie_msgs::ImpactData::ConstPtr &msg);
        void RLData_Callback(const crazyflie_msgs::RLData::ConstPtr &msg);

        void stabilizerLoop();

        crazyflie_msgs::MS MS_msg;


};

void Controller::OFState_Callback(const crazyflie_msgs::OF_SensorData::ConstPtr &msg)
{
    sensorData.Tau = msg->Tau;
    sensorData.OFx = msg->OFx;
    sensorData.OFy = msg->OFy;
    sensorData.RREV = msg->RREV;

}

void Controller::imuState_Callback(const sensor_msgs::Imu::ConstPtr &msg)
{
    sensorData.acc.x = msg->linear_acceleration.x/9.8066; // Convert to Gs to match crazyflie sensors
    sensorData.acc.y = msg->linear_acceleration.y/9.8066;
    sensorData.acc.z = msg->linear_acceleration.z/9.8066;

}

void Controller::viconState_Callback(const nav_msgs::Odometry::ConstPtr &msg)
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

void Controller::CMD_Callback(const crazyflie_msgs::RLCmd::ConstPtr &msg)
{
    setpoint.cmd_type = msg->cmd_type;
    setpoint.cmd_val1 = msg->cmd_vals.x;
    setpoint.cmd_val2 = msg->cmd_vals.y;
    setpoint.cmd_val3 = msg->cmd_vals.z;
    setpoint.cmd_flag = msg->cmd_flag;

    setpoint.GTC_cmd_rec = true;
}

void Controller::ceilingFT_Callback(const crazyflie_msgs::ImpactData::ConstPtr &msg)
{
    // _impact_flag = msg->impact_flag;
}

void Controller::RLData_Callback(const crazyflie_msgs::RLData::ConstPtr &msg){

    // if (msg->reset_flag == true){

    //     controllerGTCReset();

    // }
}