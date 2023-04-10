#pragma once

// STANDARD Includes
#include <iostream>
#include <thread>
#include <stdio.h>
#include <string>

// FIRMWARE INCLUDES
#include "app.h"
#include "controller.h"
#include "shared_lib.h"


// ROS INCLUDES
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include "crazyflie_msgs/GTC_Cmd_srv.h"



class Controller
{
    public:

        Controller(ros::NodeHandle *nh)
        {
            // ===========================
            //     ROS TOPICS/SERVICES
            // ===========================

            // INTERNAL SENSOR SUBSCRIBERS
            SAR_IMU_Subscriber = nh->subscribe("/SAR_Internal/IMU",1,&Controller::IMU_Update_Callback,this,ros::TransportHints().tcpNoDelay());
            
            // EXTERNAL SENSOR SUBSCRIBERS
            Ext_Pos_Subscriber = nh->subscribe("/SAR_External/ExtPosition",1,&Controller::Ext_Pos_Update_Callback,this,ros::TransportHints().tcpNoDelay());
            
            // MISC SERVICES
            CMD_Service = nh->advertiseService("/CTRL/Cmd_ctrl",&Controller::CMD_Service_Resp,this);


            // Thread main controller loop so other callbacks can work fine
            appThread = std::thread(&Controller::appLoop, this);
            controllerThread = std::thread(&Controller::stabilizerLoop, this);

        }

        // INTERNAL SENSOR SUBSCRIBERS
        ros::Subscriber SAR_IMU_Subscriber;

        // EXTERNAL SENSOR SUBSCRIBERS
        ros::Subscriber Ext_Pos_Subscriber;

        // SERVICES
        ros::ServiceServer CMD_Service;


        // DEFINE THREAD OBJECTS
        std::thread appThread;
        std::thread controllerThread;

        // FIRMWARE VARIABLES FOR CONTROLLER
        setpoint_t setpoint;
        sensorData_t sensorData;
        state_t state;
        control_t control;


        uint32_t tick = 1;

        // FUNCTION PROTOTYPES
        void IMU_Update_Callback(const sensor_msgs::Imu::ConstPtr &msg);
        void Ext_Pos_Update_Callback(const nav_msgs::Odometry::ConstPtr &msg);
        bool CMD_Service_Resp(crazyflie_msgs::GTC_Cmd_srv::Request &req, crazyflie_msgs::GTC_Cmd_srv::Response &res);


        void appLoop();
        void stabilizerLoop();


};


bool Controller::CMD_Service_Resp(crazyflie_msgs::GTC_Cmd_srv::Request &req, crazyflie_msgs::GTC_Cmd_srv::Response &res)
{
    // RESPOND THE SRV WAS RECIEVED
    res.srv_Success = true;

    // UPDATE GTC_Cmd STRUCT VALUES
    GTC_Cmd.cmd_type = req.cmd_type;
    GTC_Cmd.cmd_val1 = req.cmd_vals.x;
    GTC_Cmd.cmd_val2 = req.cmd_vals.y;
    GTC_Cmd.cmd_val3 = req.cmd_vals.z;
    GTC_Cmd.cmd_flag = req.cmd_flag;
    GTC_Cmd.cmd_rx = true;

    // if(req.cmd_type == 21) // RESET ROS PARAM VALUES
    // {
    //     Controller::loadParams();

    // }
    return 1;
}


// IMU VALUES FROM MODEL SENSOR PLUGIN
void Controller::IMU_Update_Callback(const sensor_msgs::Imu::ConstPtr &msg)
{
    sensorData.acc.x = msg->linear_acceleration.x/9.8066; // Convert to Gs to match crazyflie sensors
    sensorData.acc.y = msg->linear_acceleration.y/9.8066;
    sensorData.acc.z = msg->linear_acceleration.z/9.8066;

    sensorData.gyro.x = msg->angular_velocity.x*180.0/M_PI; // Convert to deg/s to match crazyflie sensors
    sensorData.gyro.y = msg->angular_velocity.y*180.0/M_PI;
    sensorData.gyro.z = msg->angular_velocity.z*180.0/M_PI;

    state.attitudeQuaternion.x = msg->orientation.x;
    state.attitudeQuaternion.y = msg->orientation.y;
    state.attitudeQuaternion.z = msg->orientation.z;
    state.attitudeQuaternion.w = msg->orientation.w;

}

// POSE AND TWIST FROM "VICON" SYSTEM (GAZEBO WORLD FRAME)
void Controller::Ext_Pos_Update_Callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    // UPDATE POSE FROM VICON SYSTEM
    state.position.x = msg->pose.pose.position.x;
    state.position.y = msg->pose.pose.position.y;
    state.position.z = msg->pose.pose.position.z;

    // UPDATE VELOCITIES FROM VICON SYSTEM
    state.velocity.x = msg->twist.twist.linear.x;
    state.velocity.y = msg->twist.twist.linear.y;
    state.velocity.z = msg->twist.twist.linear.z;

}
