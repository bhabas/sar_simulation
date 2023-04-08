/* 
This script is the main controller loop for the crazyflie. It receives
system and sensor states from the crazyflie model/gazebo via ROS topics
which it then passes to the Geometric Tracking Controller (controller_gtc.c).
It then outputs all of the chosen values via ROS topics to the 
Crazyflie_DataConverter (CF_DC) which collates all data from multiple nodes, 
reorganizes it, then republishes it. 

All changes to controller_gtc.c should remain in terms of C so it can easily
be transferred to the Crazyflie Firmware.
*/


// C++ Includes
#include <iostream>
#include <thread>
#include <stdio.h>
#include <string>

// "Firmware" Includes
#include "stabilizer_types.h"
#include "controller_gtc.h"
#include "estimator.h"
#include "nml.h"


#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include "sensor_msgs/Image.h"


#include "crazyflie_msgs/OF_SensorData.h"
#include "crazyflie_msgs/MS.h"

#include "crazyflie_msgs/CtrlData.h"
#include "crazyflie_msgs/CtrlDebug.h"

#include "crazyflie_msgs/RLCmd.h"
#include "crazyflie_msgs/RLData.h"



// FIRMWARE VARIABLES FOR CONTROLLER
static setpoint_t setpoint;
static sensorData_t sensorData;
static state_t state;
static control_t control;

class Controller
{
    public:

        Controller(ros::NodeHandle *nh)
        {
            // ===========================
            //     ROS TOPICS/SERVICES
            // ===========================

            // CONTROLLER TOPICS
            CTRL_Data_Publisher = nh->advertise<crazyflie_msgs::CtrlData>("/CTRL/data",1);
            CTRL_Debug_Publisher = nh->advertise<crazyflie_msgs::CtrlDebug>("CTRL/debug",1);
            CMD_Service = nh->advertiseService("/CTRL/Cmd_ctrl",&Controller::CMD_Service_Resp,this);


            // INTERNAL TOPICS
            SAR_IMU_Subscriber = nh->subscribe("/SAR_Internal/IMU",1,&Controller::IMU_Update_Callback,this,ros::TransportHints().tcpNoDelay());
            Ext_Pos_Subscriber = nh->subscribe("/SAR_External/ExtPosition",1,&Controller::Ext_Pos_Update_Callback,this,ros::TransportHints().tcpNoDelay());
            // CF_Camera_Subscriber = nh->subscribe("/CF_Internal/camera/image_raw",1,&Controller::Camera_Sensor_Callback,this,ros::TransportHints().tcpNoDelay());

            // ENVIRONMENT TOPICS

            Controller::loadParams();

            // Thread main controller loop so other callbacks can work fine
            controllerThread = std::thread(&Controller::stabilizerLoop, this);
            appThread = std::thread(&Controller::appLoop,this);
        }

        

        // SUBSCRIBERS
        ros::Subscriber SAR_IMU_Subscriber;
        ros::Subscriber CF_OF_Subscriber;
        ros::Subscriber CF_Camera_Subscriber;

        ros::Subscriber RL_Data_Subscriber;

        ros::Subscriber Ext_Pos_Subscriber;
        ros::Subscriber ENV_CeilingFT_Subscriber;

        // PUBLISHERS
        ros::Publisher CF_PWM_Publisher;
        ros::Publisher CTRL_Data_Publisher;
        ros::Publisher CTRL_Debug_Publisher;

        // SERVICES
        ros::ServiceServer CMD_Service;

        // DEFINE THREAD OBJECTS
        std::thread controllerThread;
        std::thread appThread;

        uint32_t tick = 1;

        // ROS SPECIFIC VALUES
        int _slowdown_type = 0;
        bool _LANDING_SLOWDOWN_FLAG;
        float _SIM_SPEED; 
        float _SIM_SLOWDOWN_SPEED;
        float _CF_MASS;
        std::string POLICY_TYPE_STR;
        std::string _MODEL_NAME;
        bool STICKY_FLAG = false;

        // ROS PARAMS
        std::string CF_Type;
        std::string CF_Config;


        // FUNCTION PRIMITIVES
        void Ext_Pos_Update_Callback(const nav_msgs::Odometry::ConstPtr &msg);
        void IMU_Update_Callback(const sensor_msgs::Imu::ConstPtr &msg);

        void Camera_Sensor_Callback(const sensor_msgs::Image::ConstPtr &msg);
        bool CMD_Service_Resp(crazyflie_msgs::RLCmd::Request &req, crazyflie_msgs::RLCmd::Response &res);


        void stabilizerLoop();
        void appLoop();
        void loadParams();
        void consoleOuput();
        void publishCtrlData();
        void publishCtrlDebug();
        

        crazyflie_msgs::CtrlData CtrlData_msg;
        crazyflie_msgs::CtrlDebug CtrlDebug_msg;

};

bool Controller::CMD_Service_Resp(crazyflie_msgs::RLCmd::Request &req, crazyflie_msgs::RLCmd::Response &res)
{

    res.srv_Success = true;

    GTC_Cmd.cmd_type = req.cmd_type;
    GTC_Cmd.cmd_val1 = req.cmd_vals.x;
    GTC_Cmd.cmd_val2 = req.cmd_vals.y;
    GTC_Cmd.cmd_val3 = req.cmd_vals.z;
    GTC_Cmd.cmd_flag = req.cmd_flag;

    // setpoint.GTC_cmd_rec = true;

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
    
    ros::param::get("/QUAD_SETTINGS/CF_Type",CF_Type);
    ros::param::get("/QUAD_SETTINGS/CF_Config",CF_Config);
    ros::param::get("/QUAD_SETTINGS/Cam_Sensor",camera_sensor_active);
    CF_Type = "/CF_Type/" + CF_Type;
    CF_Config = "/Config/" + CF_Config;
    
    // COLLECT MODEL PARAMETERS
    ros::param::get(CF_Type + CF_Config + "/Mass",m);
    ros::param::get(CF_Type + CF_Config + "/Ixx",Ixx);
    ros::param::get(CF_Type + CF_Config + "/Iyy",Iyy);
    ros::param::get(CF_Type + CF_Config + "/Izz",Izz);

    // COLLECT CTRL GAINS
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
    CtrlDebug_msg.Motorstop_Flag = motorstop_flag;
    CtrlDebug_msg.Pos_Ctrl = (bool)kp_xf;
    CtrlDebug_msg.Vel_Ctrl = (bool)kd_xf;
    CtrlDebug_msg.Traj_Active = execute_vel_traj;
    CtrlDebug_msg.Tumble_Detection = tumble_detection;
    CtrlDebug_msg.Tumbled_Flag = tumbled;
    CtrlDebug_msg.Moment_Flag = moment_flag; 
    CtrlDebug_msg.Policy_Armed = policy_armed_flag; 
    CtrlDebug_msg.Camera_Sensor_Active = camera_sensor_active;

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

    // // OPTICAL FLOW DATA
    // CtrlData_msg.Tau = sensorData.Tau;
    // CtrlData_msg.Theta_x = sensorData.Theta_x;
    // CtrlData_msg.Theta_y = sensorData.Theta_y;
    // CtrlData_msg.D_perp = sensorData.D_perp;

    // // ESTIMATED OPTICAL FLOW DATA
    // CtrlData_msg.Tau_est = sensorData.Tau_est;
    // CtrlData_msg.Theta_x_est = sensorData.Theta_x_est;
    // CtrlData_msg.Theta_y_est = sensorData.Theta_y_est;

    CtrlData_msg.Tau_thr = Tau_thr;
    CtrlData_msg.G1 = G1;

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
