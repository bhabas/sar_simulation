#pragma once

// STANDARD Includes
#include <iostream>
#include <thread>
#include <stdio.h>
#include <string>

// FIRMWARE INCLUDES
#include "app.h"
#include "controller.h"
#include "Shared_Lib.h"


// ROS INCLUDES
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

#include "sar_msgs/CTRL_Cmd_srv.h"
#include "sar_msgs/CTRL_Data.h"
#include "sar_msgs/CTRL_Debug.h"



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
            CTRL_Data_Publisher = nh->advertise<sar_msgs::CTRL_Data>("/CTRL/data",1);
            CTRL_Debug_Publisher = nh->advertise<sar_msgs::CTRL_Debug>("CTRL/debug",1);
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
        sar_msgs::CTRL_Data CtrlData_msg;
        sar_msgs::CTRL_Debug CtrlDebug_msg;


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
        std::string SAR_Type;
        std::string SAR_Config;
        std::string Cam_Config;
        std::string POLICY_TYPE_STR;


        // QUAD GEOMETRY PARAMETERS
        std::vector<float> Prop_Front_Vec;
        std::vector<float> Prop_Rear_Vec;



        // FUNCTION PROTOTYPES
        void IMU_Update_Callback(const sensor_msgs::Imu::ConstPtr &msg);
        void Ext_Pos_Update_Callback(const nav_msgs::Odometry::ConstPtr &msg);
        bool CMD_Service_Resp(sar_msgs::CTRL_Cmd_srv::Request &req, sar_msgs::CTRL_Cmd_srv::Response &res);


        void appLoop();
        void stabilizerLoop();

        void loadParams();
        void publishCtrlData();
        void publishCtrlDebug();


};


bool Controller::CMD_Service_Resp(sar_msgs::CTRL_Cmd_srv::Request &req, sar_msgs::CTRL_Cmd_srv::Response &res)
{
    // RESPOND THE SRV WAS RECIEVED
    res.srv_Success = true;

    // UPDATE CTRL_Cmd STRUCT VALUES
    CTRL_Cmd.cmd_type = req.cmd_type;
    CTRL_Cmd.cmd_val1 = req.cmd_vals.x;
    CTRL_Cmd.cmd_val2 = req.cmd_vals.y;
    CTRL_Cmd.cmd_val3 = req.cmd_vals.z;
    CTRL_Cmd.cmd_flag = req.cmd_flag;
    CTRL_Cmd.cmd_rx = req.cmd_rx;

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

    ros::param::get("/SAR_SETTINGS/SAR_Type",SAR_Type);
    ros::param::get("/SAR_SETTINGS/SAR_Config",SAR_Config);
    ros::param::get("/CAM_SETTINGS/Cam_Config",Cam_Config);



    ros::param::get("/Cam_Config/" + Cam_Config + "/X_Offset",r_CB.x);
    ros::param::get("/Cam_Config/" + Cam_Config + "/Y_Offset",r_CB.y);
    ros::param::get("/Cam_Config/" + Cam_Config + "/Z_Offset",r_CB.z);


    
    // UPDATE INERTIAL PARAMETERS
    ros::param::get("/SAR_Type/" + SAR_Type + "/Config/" + SAR_Config + "/Ref_Mass",m);
    ros::param::get("/SAR_Type/" + SAR_Type + "/Config/" + SAR_Config + "/Ref_Ixx",Ixx);
    ros::param::get("/SAR_Type/" + SAR_Type + "/Config/" + SAR_Config + "/Ref_Iyy",Iyy);
    ros::param::get("/SAR_Type/" + SAR_Type + "/Config/" + SAR_Config + "/Ref_Izz",Izz);


    // UPDATE SYSTEM PARAMETERS
    ros::param::get("/SAR_Type/" + SAR_Type +  + "/System_Params/f_max",f_max);
    ros::param::get("/SAR_Type/" + SAR_Type +  + "/System_Params/C_tf",C_tf);


    // UPDATE PROP DISTANCES
    ros::param::get("/SAR_Type/" + SAR_Type +  + "/System_Params/Prop_Front",Prop_Front_Vec);
    Prop_14_x,Prop_14_y = Prop_Front_Vec[0],Prop_Front_Vec[1];

    ros::param::get("/SAR_Type/" + SAR_Type +  + "/System_Params/Prop_Rear",Prop_Rear_Vec);   
    Prop_23_x,Prop_23_y = Prop_Rear_Vec[0],Prop_Rear_Vec[1];



    // UPDATE LANDING SURFACE PARAMETERS
    ros::param::get("/PLANE_SETTINGS/Plane_Angle",Plane_Angle);
    ros::param::get("/PLANE_SETTINGS/Pos_X",r_PO.x);
    ros::param::get("/PLANE_SETTINGS/Pos_Y",r_PO.y);
    ros::param::get("/PLANE_SETTINGS/Pos_Z",r_PO.z);



    // UPDATE CTRL GAINS
    ros::param::get("/SAR_Type/" + SAR_Type + "/CtrlGains/P_kp_xy",P_kp_xy);
    ros::param::get("/SAR_Type/" + SAR_Type + "/CtrlGains/P_kd_xy",P_kd_xy);
    ros::param::get("/SAR_Type/" + SAR_Type + "/CtrlGains/P_ki_xy",P_ki_xy);
    ros::param::get("/SAR_Type/" + SAR_Type + "/CtrlGains/i_range_xy",i_range_xy);

    ros::param::get("/SAR_Type/" + SAR_Type + "/CtrlGains/P_kp_z",P_kp_z);
    ros::param::get("/SAR_Type/" + SAR_Type + "/CtrlGains/P_kd_z",P_kd_z);
    ros::param::get("/SAR_Type/" + SAR_Type + "/CtrlGains/P_ki_z",P_ki_z);
    ros::param::get("/SAR_Type/" + SAR_Type + "/CtrlGains/i_range_z",i_range_z);

    ros::param::get("/SAR_Type/" + SAR_Type + "/CtrlGains/R_kp_xy",R_kp_xy);
    ros::param::get("/SAR_Type/" + SAR_Type + "/CtrlGains/R_kd_xy",R_kd_xy);
    ros::param::get("/SAR_Type/" + SAR_Type + "/CtrlGains/R_ki_xy",R_ki_xy);
    ros::param::get("/SAR_Type/" + SAR_Type + "/CtrlGains/i_range_R_xy",i_range_R_xy);
    
    ros::param::get("/SAR_Type/" + SAR_Type + "/CtrlGains/R_kp_z",R_kp_z);
    ros::param::get("/SAR_Type/" + SAR_Type + "/CtrlGains/R_kd_z",R_kd_z);
    ros::param::get("/SAR_Type/" + SAR_Type + "/CtrlGains/R_ki_z",R_ki_z);
    ros::param::get("/SAR_Type/" + SAR_Type + "/CtrlGains/i_range_R_z",i_range_R_z);

    ros::param::get("/SAR_SETTINGS/Cam_Active",CamActive);


    // SIMULATION SETTINGS FROM CONFIG FILE
    ros::param::get("SAR_SETTINGS/Policy_Type",POLICY_TYPE_STR); // Set string from params file into controller
    if (strcmp(POLICY_TYPE_STR.c_str(),"PARAM_OPTIM")==0)
    {
        Policy = PARAM_OPTIM;
    }
    else if (strcmp(POLICY_TYPE_STR.c_str(),"DEEP_RL_ONBOARD")==0)
    {
        Policy = DEEP_RL_ONBOARD;
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
    CtrlDebug_msg.Policy_Armed = policy_armed_flag; 

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

    // PLANE RELATIVE STATES
    CtrlData_msg.D_perp = D_perp;
    CtrlData_msg.V_perp = V_perp;
    CtrlData_msg.V_tx = V_tx;
    CtrlData_msg.V_ty = V_ty;

    // OPTICAL FLOW DATA
    CtrlData_msg.Tau = Tau;
    CtrlData_msg.Theta_x = Theta_x;
    CtrlData_msg.Theta_y = Theta_y;
    

    // ESTIMATED OPTICAL FLOW DATA
    CtrlData_msg.Tau_est = Tau_est;
    CtrlData_msg.Theta_x_est = Theta_x_est;
    CtrlData_msg.Theta_y_est = Theta_y_est;

    // NEURAL NETWORK DATA
    CtrlData_msg.Policy_Trg_Action = Policy_Trg_Action;
    CtrlData_msg.Policy_Flip_Action = Policy_Flip_Action;

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

    // CtrlData_msg.Pose_trg.header.stamp = t_flip;             
    CtrlData_msg.Pose_trg.position.x = statePos_trg.x;
    CtrlData_msg.Pose_trg.position.y = statePos_trg.y;
    CtrlData_msg.Pose_trg.position.z = statePos_trg.z;

    CtrlData_msg.Pose_trg.orientation.x = stateQuat_trg.x;
    CtrlData_msg.Pose_trg.orientation.y = stateQuat_trg.y;
    CtrlData_msg.Pose_trg.orientation.z = stateQuat_trg.z;
    CtrlData_msg.Pose_trg.orientation.w = stateQuat_trg.w;

    CtrlData_msg.Twist_trg.linear.x = stateVel_trg.x;
    CtrlData_msg.Twist_trg.linear.y = stateVel_trg.y;
    CtrlData_msg.Twist_trg.linear.z = stateVel_trg.z;

    CtrlData_msg.Twist_trg.angular.x = stateOmega_trg.x;
    CtrlData_msg.Twist_trg.angular.y = stateOmega_trg.y;
    CtrlData_msg.Twist_trg.angular.z = stateOmega_trg.z;

    // OPTICAL FLOW DATA (FLIP)
    CtrlData_msg.Tau_trg = Tau_trg;
    CtrlData_msg.Theta_x_trg = Theta_x_trg;
    CtrlData_msg.Theta_y_trg = Theta_y_trg;
    CtrlData_msg.D_perp_trg = D_perp_trg;

    // POLICY ACTIONS (FLIP)
    CtrlData_msg.Policy_Trg_Action_trg = Policy_Trg_Action_trg;
    CtrlData_msg.Policy_Flip_Action_trg = Policy_Flip_Action_trg;

    // CONTROL ACTIONS (FLIP)
    CtrlData_msg.FM_flip = {F_thrust_flip,M_x_flip,M_y_flip,M_z_flip};

    
    CTRL_Data_Publisher.publish(CtrlData_msg);

}
