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

            // RL TOPICS
            RL_CMD_Subscriber = nh->subscribe("/RL/cmd",5,&Controller::RL_CMD_Callback,this,ros::TransportHints().tcpNoDelay());

            // INTERNAL TOPICS
            CF_IMU_Subscriber = nh->subscribe("/CF_Internal/IMU",1,&Controller::IMU_Sensor_Callback,this,ros::TransportHints().tcpNoDelay());
            CF_OF_Subscriber = nh->subscribe("/CF_Internal/OF_Sensor",1,&Controller::OF_Sensor_Callback,this,ros::TransportHints().tcpNoDelay());
            CF_PWM_Publisher = nh->advertise<crazyflie_msgs::MS>("/CF_Internal/MS_PWM",1);

            // ENVIRONMENT TOPICS
            ENV_Vicon_Subscriber = nh->subscribe("/ENV/viconState_UKF",1,&Controller::viconState_Callback,this,ros::TransportHints().tcpNoDelay());
            
            Controller::loadParams();

            // Thread main controller loop so other callbacks can work fine
            controllerThread = std::thread(&Controller::stabilizerLoop, this);
        }

        

        // SUBSCRIBERS
        ros::Subscriber CF_IMU_Subscriber;
        ros::Subscriber CF_OF_Subscriber;

        ros::Subscriber RL_CMD_Subscriber;
        ros::Subscriber RL_Data_Subscriber;

        ros::Subscriber ENV_Vicon_Subscriber;
        ros::Subscriber ENV_CeilingFT_Subscriber;

        // PUBLISHERS
        ros::Publisher CF_PWM_Publisher;
        ros::Publisher CTRL_Data_Publisher;
        ros::Publisher CTRL_Debug_Publisher;

        // SERVICES
        ros::ServiceClient GZ_SimSpeed_Client;

        // DEFINE THREAD OBJECTS
        std::thread controllerThread;

        uint32_t tick = 1;

        // ROS SPECIFIC VALUES
        int _slowdown_type = 0;
        float _H_CEILING = 2.10;
        bool _LANDING_SLOWDOWN_FLAG;
        float _SIM_SPEED; 
        float _SIM_SLOWDOWN_SPEED;
        float _CF_MASS;
        int _POLICY_TYPE;
        std::string _MODEL_NAME;
        bool STICKY_FLAG = false;

        // FUNCTION PRIMITIVES
        void viconState_Callback(const nav_msgs::Odometry::ConstPtr &msg);
        void IMU_Sensor_Callback(const sensor_msgs::Imu::ConstPtr &msg);
        void OF_Sensor_Callback(const crazyflie_msgs::OF_SensorData::ConstPtr &msg);
        void RL_CMD_Callback(const crazyflie_msgs::RLCmd::ConstPtr &msg);

        void stabilizerLoop();
        void loadParams();
        void consoleOuput();
        void publishCtrlData();
        void publishCtrlDebug();
        

        crazyflie_msgs::MS MS_PWM_msg;
        crazyflie_msgs::CtrlData CtrlData_msg;
        crazyflie_msgs::CtrlDebug CtrlDebug_msg;

};



// OPTICAL FLOW VALUES (IN BODY FRAME) FROM MODEL SENSOR PLUGIN
void Controller::OF_Sensor_Callback(const crazyflie_msgs::OF_SensorData::ConstPtr &msg)
{
    sensorData.Tau = msg->Tau;
    sensorData.OFx = msg->OFx;
    sensorData.OFy = msg->OFy;
    sensorData.RREV = msg->RREV;
    sensorData.d_ceil = msg->d_ceil;

}

// IMU VALUES FROM MODEL SENSOR PLUGIN
void Controller::IMU_Sensor_Callback(const sensor_msgs::Imu::ConstPtr &msg)
{
    sensorData.acc.x = msg->linear_acceleration.x/9.8066; // Convert to Gs to match crazyflie sensors
    sensorData.acc.y = msg->linear_acceleration.y/9.8066;
    sensorData.acc.z = msg->linear_acceleration.z/9.8066;

}

// POSE AND TWIST FROM "VICON" SYSTEM (GAZEBO WORLD FRAME)
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

// RECEIVE COMMANDS FROM RL SCRIPTS
void Controller::RL_CMD_Callback(const crazyflie_msgs::RLCmd::ConstPtr &msg)
{
    setpoint.cmd_type = msg->cmd_type;
    setpoint.cmd_val1 = msg->cmd_vals.x;
    setpoint.cmd_val2 = msg->cmd_vals.y;
    setpoint.cmd_val3 = msg->cmd_vals.z;
    setpoint.cmd_flag = msg->cmd_flag;

    setpoint.GTC_cmd_rec = true;

    if(msg->cmd_type == 6) // RESET ROS PARAM VALUES
    {
        Controller::loadParams();

    }

}


// LOAD VALUES FROM ROSPARAM SERVER
void Controller::loadParams()
{
    // SIMULATION SETTINGS FROM CONFIG FILE
    // ros::param::get("/MODEL_NAME",_MODEL_NAME);
    // ros::param::get("/CEILING_HEIGHT",_H_CEILING);
    ros::param::get("/CF_MASS",_CF_MASS);
    ros::param::get("/POLICY_TYPE",_POLICY_TYPE);
    POLICY_TYPE = (Policy_Type)_POLICY_TYPE; // Cast ROS param (int) to enum (Policy_Type)

    // DEBUG SETTINGS
    // ros::param::get("/SIM_SPEED",_SIM_SPEED);
    // ros::param::get("/SIM_SLOWDOWN_SPEED",_SIM_SLOWDOWN_SPEED);
    // ros::param::get("/LANDING_SLOWDOWN_FLAG",_LANDING_SLOWDOWN_FLAG);

    // COLLECT CTRL GAINS FROM CONFIG FILE
    ros::param::get("P_kp_xy",P_kp_xy);
    ros::param::get("P_kd_xy",P_kd_xy);
    ros::param::get("P_ki_xy",P_ki_xy);
    ros::param::get("i_range_xy",i_range_xy);

    ros::param::get("P_kp_z",P_kp_z);
    ros::param::get("P_kd_z",P_kd_z);
    ros::param::get("P_ki_z",P_ki_z);
    ros::param::get("i_range_z",i_range_z);

    ros::param::get("R_kp_xy",R_kp_xy);
    ros::param::get("R_kd_xy",R_kd_xy);
    ros::param::get("R_ki_xy",R_ki_xy);
    ros::param::get("i_range_R_xy",i_range_R_xy);
    
    ros::param::get("R_kp_z",R_kp_z);
    ros::param::get("R_kd_z",R_kd_z);
    ros::param::get("R_ki_z",R_ki_z);
    ros::param::get("i_range_R_z",i_range_R_z);

}


void Controller::publishCtrlDebug()
{
    CtrlDebug_msg.Motorstop_Flag = motorstop_flag;
    CtrlDebug_msg.Pos_Ctrl = (bool)kp_xf;
    CtrlDebug_msg.Vel_Ctrl = (bool)kd_xf;
    CtrlDebug_msg.Traj_Active = execute_traj;
    CtrlDebug_msg.Tumble_Detection = tumble_detection;
    CtrlDebug_msg.Tumbled_Flag = tumbled;
    CtrlDebug_msg.Moment_Flag = moment_flag; 
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

    // OPTICAL FLOW DATA
    CtrlData_msg.Tau = Tau;
    CtrlData_msg.OFx = OFx;
    CtrlData_msg.OFy = OFy;
    CtrlData_msg.RREV = RREV;
    CtrlData_msg.D_ceil = d_ceil;

    CtrlData_msg.Tau_thr = Tau_thr;
    CtrlData_msg.G1 = G1;

    // NEURAL NETWORK DATA
    CtrlData_msg.NN_policy = NN_policy;
    CtrlData_msg.NN_flip = NN_flip;

    // CONTROL ACTIONS
    CtrlData_msg.FM = {F_thrust,M.x*1.0e3,M.y*1.0e3,M.z*1.0e3};
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
    CtrlData_msg.OFx_tr = OFx_tr;
    CtrlData_msg.OFy_tr = OFy_tr;
    CtrlData_msg.RREV_tr = RREV_tr;
    CtrlData_msg.D_ceil_tr = d_ceil_tr;

    // NEURAL NETWORK DATA (FLIP)
    CtrlData_msg.NN_tr_flip = NN_tr_flip;
    CtrlData_msg.NN_tr_policy = NN_tr_policy;

    // CONTROL ACTIONS (FLIP)
    CtrlData_msg.FM_flip = {F_thrust_flip,M_x_flip*1.0e3,M_y_flip*1.0e3,M_z_flip*1.0e3};

    
    CTRL_Data_Publisher.publish(CtrlData_msg);

}