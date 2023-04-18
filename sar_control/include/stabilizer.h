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
#include "crazyflie_msgs/CtrlData.h"
#include "crazyflie_msgs/CtrlDebug.h"



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
            
            // MISC SERVICES/PUBLISHERS
            CTRL_Data_Publisher = nh->advertise<crazyflie_msgs::CtrlData>("/CTRL/data",1);
            CTRL_Debug_Publisher = nh->advertise<crazyflie_msgs::CtrlDebug>("CTRL/debug",1);
            CTRL_CMD_Service = nh->advertiseService("/CTRL/Cmd_ctrl",&Controller::CMD_Service_Resp,this);



            // Thread main controller loop so other callbacks can work fine
            appThread = std::thread(&Controller::appLoop, this);
            controllerThread = std::thread(&Controller::stabilizerLoop, this);

        }

        // INTERNAL SENSOR SUBSCRIBERS
        ros::Subscriber SAR_IMU_Subscriber;

        // EXTERNAL SENSOR SUBSCRIBERS
        ros::Subscriber Ext_Pos_Subscriber;

        // SERVICES
        ros::ServiceServer CTRL_CMD_Service;
        ros::Publisher CTRL_Data_Publisher;
        ros::Publisher CTRL_Debug_Publisher;

        // MESSAGES
        crazyflie_msgs::CtrlData CtrlData_msg;
        crazyflie_msgs::CtrlDebug CtrlDebug_msg;


        // DEFINE THREAD OBJECTS
        std::thread appThread;
        std::thread controllerThread;

        // FIRMWARE VARIABLES FOR CONTROLLER
        setpoint_t setpoint;
        sensorData_t sensorData;
        state_t state;
        control_t control;
        uint32_t tick = 1;

        // ROS PARAMS
        std::string CF_Type;
        std::string CF_Config;
        std::string POLICY_TYPE_STR;


        // FUNCTION PROTOTYPES
        void IMU_Update_Callback(const sensor_msgs::Imu::ConstPtr &msg);
        void Ext_Pos_Update_Callback(const nav_msgs::Odometry::ConstPtr &msg);
        bool CMD_Service_Resp(crazyflie_msgs::GTC_Cmd_srv::Request &req, crazyflie_msgs::GTC_Cmd_srv::Response &res);


        void appLoop();
        void stabilizerLoop();

        void loadParams();
        void publishCtrlData();
        void publishCtrlDebug();


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
    GTC_Cmd.cmd_rx = req.cmd_rx;

    if(req.cmd_type == 21) // RESET ROS PARAM VALUES
    {
        Controller::loadParams();

    }

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

// LOAD VALUES FROM ROSPARAM SERVER INTO CONTROLLER
void Controller::loadParams()
{
    printf("Updating Parameters\n");

    ros::param::get("/QUAD_SETTINGS/CF_Type",CF_Type);
    ros::param::get("/QUAD_SETTINGS/CF_Config",CF_Config);
    CF_Type = "/CF_Type/" + CF_Type;
    CF_Config = "/Config/" + CF_Config;
    
    // UPDATE INERTIAL PARAMETERS
    ros::param::get(CF_Type + CF_Config + "/Mass",m);
    ros::param::get(CF_Type + CF_Config + "/Ixx",Ixx);
    ros::param::get(CF_Type + CF_Config + "/Iyy",Iyy);
    ros::param::get(CF_Type + CF_Config + "/Izz",Izz);

    // UPDATE SYSTEM PARAMETERS
    ros::param::get(CF_Type + CF_Config + "/System_Params/f_max",f_max);
    ros::param::get(CF_Type + CF_Config + "/System_Params/C_tf",C_tf);
    ros::param::get(CF_Type + CF_Config + "/System_Params/Prop_Dist",Prop_Dist);


    // UPDATE CTRL GAINS
    ros::param::get(CF_Type + "/CtrlGains/P_kp_xy",P_kp_xy);
    ros::param::get(CF_Type + "/CtrlGains/P_kd_xy",P_kd_xy);
    ros::param::get(CF_Type + "/CtrlGains/P_ki_xy",P_ki_xy);
    ros::param::get(CF_Type + "/CtrlGains/i_range_xy",i_range_xy);

    ros::param::get(CF_Type + "/CtrlGains/P_kp_z",P_kp_z);
    ros::param::get(CF_Type + "/CtrlGains/P_kd_z",P_kd_z);
    ros::param::get(CF_Type + "/CtrlGains/P_ki_z",P_ki_z);
    ros::param::get(CF_Type + "/CtrlGains/i_range_z",i_range_z);

    ros::param::get(CF_Type + "/CtrlGains/R_kp_xy",R_kp_xy);
    ros::param::get(CF_Type + "/CtrlGains/R_kd_xy",R_kd_xy);
    ros::param::get(CF_Type + "/CtrlGains/R_ki_xy",R_ki_xy);
    ros::param::get(CF_Type + "/CtrlGains/i_range_R_xy",i_range_R_xy);
    
    ros::param::get(CF_Type + "/CtrlGains/R_kp_z",R_kp_z);
    ros::param::get(CF_Type + "/CtrlGains/R_kd_z",R_kd_z);
    ros::param::get(CF_Type + "/CtrlGains/R_ki_z",R_ki_z);
    ros::param::get(CF_Type + "/CtrlGains/i_range_R_z",i_range_R_z);

    // ros::param::get("/QUAD_SETTINGS/Cam_Sensor",camera_sensor_active);


    // SIMULATION SETTINGS FROM CONFIG FILE
    ros::param::get("QUAD_SETTINGS/Policy_Type",POLICY_TYPE_STR); // Set string from params file into controller
    if (strcmp(POLICY_TYPE_STR.c_str(),"PARAM_OPTIM")==0)
    {
        Policy = PARAM_OPTIM;
    }
    else if (strcmp(POLICY_TYPE_STR.c_str(),"SVL_POLICY")==0)
    {
        Policy = SVL_POLICY;
    }
    else if (strcmp(POLICY_TYPE_STR.c_str(),"DEEP_RL")==0)
    {
        Policy = DEEP_RL;
    }    
    else if (strcmp(POLICY_TYPE_STR.c_str(),"DEEP_RL_SB3")==0)
    {
        Policy = DEEP_RL_SB3;
    }    

}


void Controller::publishCtrlDebug()
{
    CtrlDebug_msg.Pos_Ctrl = (bool)kp_xf;
    CtrlDebug_msg.Vel_Ctrl = (bool)kd_xf;
    CtrlDebug_msg.Tumble_Detection = tumble_detection;
    CtrlDebug_msg.Tumbled_Flag = tumbled;
    CtrlDebug_msg.Moment_Flag = moment_flag; 
    CtrlDebug_msg.Motorstop_Flag = motorstop_flag;


    // CtrlDebug_msg.Traj_Active = execute_vel_traj;
    // CtrlDebug_msg.Policy_Armed = policy_armed_flag; 
    // CtrlDebug_msg.Camera_Sensor_Active = camera_sensor_active;

    CTRL_Debug_Publisher.publish(CtrlDebug_msg);
}

// PUBLISH CONTROLLER DATA ON ROS TOPIC
void Controller::publishCtrlData()
{
    // STATE DATA
    CtrlData_msg.Pose.position.x = statePos.x;
    CtrlData_msg.Pose.position.y = statePos.y;
    CtrlData_msg.Pose.position.z = statePos.z;

    CtrlData_msg.Pose.orientation.x = stateQuat.x;
    CtrlData_msg.Pose.orientation.y = stateQuat.y;
    CtrlData_msg.Pose.orientation.z = stateQuat.z;
    CtrlData_msg.Pose.orientation.w = stateQuat.w;

    CtrlData_msg.Twist.linear.x = stateVel.x;
    CtrlData_msg.Twist.linear.y = stateVel.y;
    CtrlData_msg.Twist.linear.z = stateVel.z;

    CtrlData_msg.Twist.angular.x = stateOmega.x;
    CtrlData_msg.Twist.angular.y = stateOmega.y;
    CtrlData_msg.Twist.angular.z = stateOmega.z;

    // OPTICAL FLOW DATA
    CtrlData_msg.Tau = Tau;
    CtrlData_msg.Theta_x = Theta_x;
    CtrlData_msg.Theta_y = Theta_y;
    CtrlData_msg.D_perp = D_perp;

    // ESTIMATED OPTICAL FLOW DATA
    CtrlData_msg.Tau_est = Tau_est;
    CtrlData_msg.Theta_x_est = Theta_x_est;
    CtrlData_msg.Theta_y_est = Theta_y_est;

    // CtrlData_msg.Tau_thr = Tau_thr;
    // CtrlData_msg.G1 = G1;

    // NEURAL NETWORK DATA
    CtrlData_msg.Policy_Flip = Policy_Flip;
    CtrlData_msg.Policy_Action = Policy_Action;

    // CONTROL ACTIONS
    CtrlData_msg.FM = {F_thrust,M.x*1.0e3,M.y*1.0e3,M.z*1.0e3};
    CtrlData_msg.MotorThrusts = {M1_thrust,M2_thrust,M3_thrust,M4_thrust};
    CtrlData_msg.MS_PWM = {M1_pwm,M2_pwm,M3_pwm,M4_pwm};

    CtrlData_msg.x_d.x = x_d.x;
    CtrlData_msg.x_d.y = x_d.y;
    CtrlData_msg.x_d.z = x_d.z;

    CtrlData_msg.v_d.x = v_d.x;
    CtrlData_msg.v_d.y = v_d.y;
    CtrlData_msg.v_d.z = v_d.z;

    CtrlData_msg.a_d.x = a_d.x;
    CtrlData_msg.a_d.y = a_d.y;
    CtrlData_msg.a_d.z = a_d.z;



    // STATE DATA (FLIP)
    CtrlData_msg.flip_flag = flip_flag;

    // CtrlData_msg.Pose_tr.header.stamp = t_flip;             
    CtrlData_msg.Pose_tr.position.x = statePos_tr.x;
    CtrlData_msg.Pose_tr.position.y = statePos_tr.y;
    CtrlData_msg.Pose_tr.position.z = statePos_tr.z;

    CtrlData_msg.Pose_tr.orientation.x = stateQuat_tr.x;
    CtrlData_msg.Pose_tr.orientation.y = stateQuat_tr.y;
    CtrlData_msg.Pose_tr.orientation.z = stateQuat_tr.z;
    CtrlData_msg.Pose_tr.orientation.w = stateQuat_tr.w;

    CtrlData_msg.Twist_tr.linear.x = stateVel_tr.x;
    CtrlData_msg.Twist_tr.linear.y = stateVel_tr.y;
    CtrlData_msg.Twist_tr.linear.z = stateVel_tr.z;

    CtrlData_msg.Twist_tr.angular.x = stateOmega_tr.x;
    CtrlData_msg.Twist_tr.angular.y = stateOmega_tr.y;
    CtrlData_msg.Twist_tr.angular.z = stateOmega_tr.z;

    // OPTICAL FLOW DATA (FLIP)
    CtrlData_msg.Tau_tr = Tau_tr;
    CtrlData_msg.Theta_x_tr = Theta_x_tr;
    CtrlData_msg.Theta_y_tr = Theta_y_tr;
    CtrlData_msg.D_perp_tr = D_perp_tr;

    // NEURAL NETWORK DATA (FLIP)
    CtrlData_msg.Policy_Flip_tr = Policy_Flip_tr;
    CtrlData_msg.Policy_Action_tr = Policy_Action_tr;

    // CONTROL ACTIONS (FLIP)
    CtrlData_msg.FM_flip = {F_thrust_flip,M_x_flip,M_y_flip,M_z_flip};

    
    CTRL_Data_Publisher.publish(CtrlData_msg);

}
