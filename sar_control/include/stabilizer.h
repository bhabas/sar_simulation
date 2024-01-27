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
#include "sar_msgs/SAR_MiscData.h"




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
            SAR_DC_Subscriber = nh->subscribe("/SAR_DC/MiscData",1,&Controller::Plane_Pose_Callback,this,ros::TransportHints().tcpNoDelay());



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
        ros::Subscriber SAR_DC_Subscriber;

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
        void Plane_Pose_Callback(const sar_msgs::SAR_MiscData::ConstPtr &msg);


        void appLoop();
        void stabilizerLoop();

        void loadParams();
        void publishCtrlData();
        void publishCtrlDebug();


};

void Controller::Plane_Pose_Callback(sar_msgs::SAR_MiscData::ConstPtr const &msg)
{
    Plane_Angle_deg = msg->Plane_Angle;
    r_P_O.x = msg->Plane_Pos.x;
    r_P_O.y = msg->Plane_Pos.y;
    r_P_O.z = msg->Plane_Pos.z;
}

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
    sensorData.acc.x = -msg->linear_acceleration.x/9.8066; // Convert to Gs to match crazyflie sensors
    sensorData.acc.y = -msg->linear_acceleration.y/9.8066;
    sensorData.acc.z = msg->linear_acceleration.z/9.8066;

    sensorData.gyro.x = msg->angular_velocity.x*180.0/M_PI; // Convert to deg/s to match crazyflie sensors
    sensorData.gyro.y = msg->angular_velocity.y*180.0/M_PI;
    sensorData.gyro.z = msg->angular_velocity.z*180.0/M_PI;

    state.attitudeQuaternion.x = msg->orientation.x;
    state.attitudeQuaternion.y = msg->orientation.y;
    state.attitudeQuaternion.z = msg->orientation.z;
    state.attitudeQuaternion.w = msg->orientation.w;

    state.acc.x = -msg->linear_acceleration.x/9.8066; // Convert to Gs to match crazyflie sensors
    state.acc.y = -msg->linear_acceleration.y/9.8066;
    state.acc.z = msg->linear_acceleration.z/9.8066;

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

    
    // UPDATE INERTIAL PARAMETERS
    ros::param::get("/SAR_Type/" + SAR_Type + "/Config/" + SAR_Config + "/Ref_Mass",m);
    ros::param::get("/SAR_Type/" + SAR_Type + "/Config/" + SAR_Config + "/Ref_Ixx",Ixx);
    ros::param::get("/SAR_Type/" + SAR_Type + "/Config/" + SAR_Config + "/Ref_Iyy",Iyy);
    ros::param::get("/SAR_Type/" + SAR_Type + "/Config/" + SAR_Config + "/Ref_Izz",Izz);


    // UPDATE SYSTEM PARAMETERS
    ros::param::get("/SAR_Type/" + SAR_Type + "/System_Params/f_max",f_max);
    ros::param::get("/SAR_Type/" + SAR_Type + "/System_Params/C_tf",C_tf);


    // UPDATE PROP DISTANCES
    ros::param::get("/SAR_Type/" + SAR_Type + "/System_Params/Prop_Front",Prop_Front_Vec);
    Prop_14_x = Prop_Front_Vec[0];
    Prop_14_y = Prop_Front_Vec[1];


    ros::param::get("/SAR_Type/" + SAR_Type + "/System_Params/Prop_Rear",Prop_Rear_Vec);   
    Prop_23_x = Prop_Rear_Vec[0];
    Prop_23_y = Prop_Rear_Vec[1];


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

    ros::param::get("/SAR_SETTINGS/Cam_Active",CamActive_Flag);


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
    CtrlDebug_msg.Tumbled_Flag = Tumbled_Flag;
    CtrlDebug_msg.TumbleDetect_Flag = TumbleDetect_Flag;
    CtrlDebug_msg.MotorStop_Flag = MotorStop_Flag;
    CtrlDebug_msg.AngAccel_Flag = AngAccel_Flag; 
    CtrlDebug_msg.SafeMode_Flag = SafeMode_Flag;
    CtrlDebug_msg.CustomThrust_Flag = CustomThrust_Flag;
    CtrlDebug_msg.CustomPWM_Flag = CustomPWM_Flag;

    CtrlDebug_msg.Pos_Ctrl_Flag = (bool)kp_xf;
    CtrlDebug_msg.Vel_Ctrl_Flag = (bool)kd_xf;
    CtrlDebug_msg.Policy_Armed_Flag = Policy_Armed_Flag; 
    CtrlDebug_msg.CamActive_Flag = CamActive_Flag;

    CTRL_Debug_Publisher.publish(CtrlDebug_msg);
}

// PUBLISH CONTROLLER DATA ON ROS TOPIC
void Controller::publishCtrlData()
{

    // STATE DATA WRT ORIGIN
    CtrlData_msg.Pose_B_O.position.x = Pos_B_O.x;
    CtrlData_msg.Pose_B_O.position.y = Pos_B_O.y;
    CtrlData_msg.Pose_B_O.position.z = Pos_B_O.z;

    CtrlData_msg.Pose_B_O.orientation.x = Quat_B_O.x;
    CtrlData_msg.Pose_B_O.orientation.y = Quat_B_O.y;
    CtrlData_msg.Pose_B_O.orientation.z = Quat_B_O.z;
    CtrlData_msg.Pose_B_O.orientation.w = Quat_B_O.w;

    CtrlData_msg.Twist_B_O.linear.x = Vel_B_O.x;
    CtrlData_msg.Twist_B_O.linear.y = Vel_B_O.y;
    CtrlData_msg.Twist_B_O.linear.z = Vel_B_O.z;

    CtrlData_msg.Twist_B_O.angular.x = Omega_B_O.x;
    CtrlData_msg.Twist_B_O.angular.y = Omega_B_O.y;
    CtrlData_msg.Twist_B_O.angular.z = Omega_B_O.z;

    CtrlData_msg.Accel_B_O.linear.x = Accel_B_O.x;
    CtrlData_msg.Accel_B_O.linear.y = Accel_B_O.y;
    CtrlData_msg.Accel_B_O.linear.z = Accel_B_O.z;

    CtrlData_msg.Accel_B_O.angular.x = dOmega_B_O.x;
    CtrlData_msg.Accel_B_O.angular.y = dOmega_B_O.y;
    CtrlData_msg.Accel_B_O.angular.z = dOmega_B_O.z;

    CtrlData_msg.Accel_B_O_Mag = Accel_B_O_Mag;


    // STATE DATA WRT PLANE
    CtrlData_msg.Pose_P_B.position.x = Pos_P_B.x;
    CtrlData_msg.Pose_P_B.position.y = Pos_P_B.y;
    CtrlData_msg.Pose_P_B.position.z = Pos_P_B.z;

    CtrlData_msg.Pose_P_B.orientation.x = Quat_P_B.x;
    CtrlData_msg.Pose_P_B.orientation.y = Quat_P_B.y;
    CtrlData_msg.Pose_P_B.orientation.z = Quat_P_B.z;
    CtrlData_msg.Pose_P_B.orientation.w = Quat_P_B.w;

    CtrlData_msg.Twist_B_P.linear.x = Vel_B_P.x;
    CtrlData_msg.Twist_B_P.linear.y = Vel_B_P.y;
    CtrlData_msg.Twist_B_P.linear.z = Vel_B_P.z;

    CtrlData_msg.Twist_B_P.angular.x = Omega_B_P.x;
    CtrlData_msg.Twist_B_P.angular.y = Omega_B_P.y;
    CtrlData_msg.Twist_B_P.angular.z = Omega_B_P.z;


    // PLANE RELATIVE STATES
    CtrlData_msg.D_perp = D_perp;
    CtrlData_msg.Vel_mag_B_P = Vel_mag_B_P;
    CtrlData_msg.Vel_angle_B_P = Vel_angle_B_P;

    // OPTICAL FLOW DATA
    CtrlData_msg.Optical_Flow.x = Theta_x;
    CtrlData_msg.Optical_Flow.y = Theta_y;
    CtrlData_msg.Optical_Flow.z = Tau;
    

    // ESTIMATED OPTICAL FLOW DATA
    CtrlData_msg.Optical_Flow_Cam.x = Theta_x_Cam;
    CtrlData_msg.Optical_Flow_Cam.y = Theta_y_Cam;
    CtrlData_msg.Optical_Flow_Cam.z = Tau_Cam;

    // POLICY ACTIONS
    CtrlData_msg.Policy_Trg_Action = Policy_Trg_Action;
    CtrlData_msg.Policy_Rot_Action = Policy_Rot_Action;

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



    // ==========================
    //  STATES AT POLICY TRIGGER
    // ==========================
    CtrlData_msg.Trg_Flag = Trg_Flag;

    // STATE DATA WRT ORIGIN
    CtrlData_msg.Pose_B_O_trg.position.x = Pos_B_O_trg.x;
    CtrlData_msg.Pose_B_O_trg.position.y = Pos_B_O_trg.y;
    CtrlData_msg.Pose_B_O_trg.position.z = Pos_B_O_trg.z;

    CtrlData_msg.Pose_B_O_trg.orientation.x = Quat_B_O_trg.x;
    CtrlData_msg.Pose_B_O_trg.orientation.y = Quat_B_O_trg.y;
    CtrlData_msg.Pose_B_O_trg.orientation.z = Quat_B_O_trg.z;
    CtrlData_msg.Pose_B_O_trg.orientation.w = Quat_B_O_trg.w;

    CtrlData_msg.Twist_B_O_trg.linear.x = Vel_B_O_trg.x;
    CtrlData_msg.Twist_B_O_trg.linear.y = Vel_B_O_trg.y;
    CtrlData_msg.Twist_B_O_trg.linear.z = Vel_B_O_trg.z;

    CtrlData_msg.Twist_B_O_trg.angular.x = Omega_B_O_trg.x;
    CtrlData_msg.Twist_B_O_trg.angular.y = Omega_B_O_trg.y;
    CtrlData_msg.Twist_B_O_trg.angular.z = Omega_B_O_trg.z;

    // STATE DATA WRT PLANE
    CtrlData_msg.Pose_P_B_trg.position.x = Pos_P_B_trg.x;
    CtrlData_msg.Pose_P_B_trg.position.y = Pos_P_B_trg.y;
    CtrlData_msg.Pose_P_B_trg.position.z = Pos_P_B_trg.z;

    CtrlData_msg.Pose_P_B_trg.orientation.x = Quat_P_B_trg.x;
    CtrlData_msg.Pose_P_B_trg.orientation.y = Quat_P_B_trg.y;
    CtrlData_msg.Pose_P_B_trg.orientation.z = Quat_P_B_trg.z;
    CtrlData_msg.Pose_P_B_trg.orientation.w = Quat_P_B_trg.w;

    CtrlData_msg.Twist_B_P_trg.linear.x = Vel_B_P_trg.x;
    CtrlData_msg.Twist_B_P_trg.linear.y = Vel_B_P_trg.y;    
    CtrlData_msg.Twist_B_P_trg.linear.z = Vel_B_P_trg.z;

    CtrlData_msg.Twist_B_P_trg.angular.x = Omega_B_P_trg.x;
    CtrlData_msg.Twist_B_P_trg.angular.y = Omega_B_P_trg.y;
    CtrlData_msg.Twist_B_P_trg.angular.z = Omega_B_P_trg.z;


    // OPTICAL FLOW DATA (TRIGGER)
    CtrlData_msg.Optical_Flow_trg.x = Theta_x_trg;
    CtrlData_msg.Optical_Flow_trg.y = Theta_y_trg;
    CtrlData_msg.Optical_Flow_trg.z = Tau_trg;


    // POLICY ACTIONS (TRIGGER)
    CtrlData_msg.Policy_Trg_Action_trg = Policy_Trg_Action_trg;
    CtrlData_msg.Policy_Rot_Action_trg = Policy_Rot_Action_trg;

    // ==========================
    //      STATES AT IMPACT
    // ==========================
    CtrlData_msg.Impact_Flag_OB = Impact_Flag_OB;
    CtrlData_msg.Accel_B_O_Mag_impact_OB = Accel_B_O_Mag_impact_OB;

    CtrlData_msg.Pose_B_O_impact_OB.position.x = Pos_B_O_impact_OB.x;
    CtrlData_msg.Pose_B_O_impact_OB.position.y = Pos_B_O_impact_OB.y;
    CtrlData_msg.Pose_B_O_impact_OB.position.z = Pos_B_O_impact_OB.z;

    CtrlData_msg.Pose_B_O_impact_OB.orientation.x = Quat_B_O_impact_OB.x;
    CtrlData_msg.Pose_B_O_impact_OB.orientation.y = Quat_B_O_impact_OB.y;
    CtrlData_msg.Pose_B_O_impact_OB.orientation.z = Quat_B_O_impact_OB.z;
    CtrlData_msg.Pose_B_O_impact_OB.orientation.w = Quat_B_O_impact_OB.w;

    CtrlData_msg.Twist_B_P_impact_OB.linear.x = Vel_B_P_impact_OB.x;
    CtrlData_msg.Twist_B_P_impact_OB.linear.y = Vel_B_P_impact_OB.y;
    CtrlData_msg.Twist_B_P_impact_OB.linear.z = Vel_B_P_impact_OB.z;

    CtrlData_msg.Twist_B_P_impact_OB.angular.x = Omega_B_P_impact_OB.x;
    CtrlData_msg.Twist_B_P_impact_OB.angular.y = Omega_B_P_impact_OB.y;
    CtrlData_msg.Twist_B_P_impact_OB.angular.z = Omega_B_P_impact_OB.z;
    
    CTRL_Data_Publisher.publish(CtrlData_msg);
}
