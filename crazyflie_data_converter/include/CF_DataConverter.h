/* 
Data from the crazyflie is compressed and streamed to the Crazyswarm node via CRTP which then 
then directly converts the raw data into ROS topics. This script subscribes to the ROS topics 
with the raw data from the crazyflie, decompresses the data, organizes it, 
and then republishes the data in an organized manner that is easy to use.
*/

#include <stdio.h>
#include <iostream>

#include "ros/ros.h"
#include "quatcompress.h"

// MESSAGE INCLUDES
#include "crazyflie_msgs/CF_StateData.h"
#include "crazyflie_msgs/CF_FlipData.h"
#include "crazyflie_msgs/CF_ImpactData.h"
#include "crazyflie_msgs/CF_MiscData.h"
#include "crazyflie_msgs_exp/GenericLogData.h"

#include "crazyflie_msgs/CtrlData.h"
#include "crazyflie_msgs/RLCmd.h"




class CF_DataConverter
{
    public:
        // CONSTRUCTOR TO START PUBLISHERS AND SUBSCRIBERS (Similar to Python's __init__())
        CF_DataConverter(ros::NodeHandle* nh)
        {
            // INITIALIZE SUBSCRIBERS
            CTRL_Data_Subscriber = nh->subscribe("/CTRL/data", 1, &CF_DataConverter::ctrlData_Callback, this, ros::TransportHints().tcpNoDelay());
            RL_CMD_Subscriber = nh->subscribe("/RL/cmd",5,&CF_DataConverter::RL_CMD_Callback,this,ros::TransportHints().tcpNoDelay());
            // log2_Sub = nh->subscribe("/cf1/log2", 1, &CF_DataConverter::log2_Callback, this, ros::TransportHints().tcpNoDelay());
            // log3_Sub = nh->subscribe("/cf1/log3", 1, &CF_DataConverter::log3_Callback, this, ros::TransportHints().tcpNoDelay());
            // log4_Sub = nh->subscribe("/cf1/log4", 1, &CF_DataConverter::log4_Callback, this, ros::TransportHints().tcpNoDelay());

            // INITIALIZE PUBLISHERS
            StateData_Pub = nh->advertise<crazyflie_msgs::CF_StateData>("/CF_DC/StateData",1);
            MiscData_Pub =  nh->advertise<crazyflie_msgs::CF_MiscData>("/CF_DC/MiscData",1);
            FlipData_Pub =  nh->advertise<crazyflie_msgs::CF_FlipData>("/CF_DC/FlipData",1);
            ImpactData_Pub = nh->advertise<crazyflie_msgs::CF_ImpactData>("/CF_DC/ImpactData",1);
        }


        // FUNCTION PRIMITIVES
        void ctrlData_Callback(const crazyflie_msgs::CtrlData &ctrl_msg);
        void RL_CMD_Callback(const crazyflie_msgs::RLCmd::ConstPtr &msg);
        // void log2_Callback(const crazyflie_msgs_exp::GenericLogData::ConstPtr &log2_msg);
        // void log3_Callback(const crazyflie_msgs_exp::GenericLogData::ConstPtr &log3_msg);
        // void log4_Callback(const crazyflie_msgs_exp::GenericLogData::ConstPtr &log4_msg);

        void decompressXY(uint32_t xy, float xy_arr[]);
        void quat2euler(float quat[], float eul[]);

    private:

        // SUBSCRIBERS
        ros::Subscriber CTRL_Data_Subscriber;
        ros::Subscriber RL_CMD_Subscriber;
        ros::Subscriber log3_Sub;
        ros::Subscriber log4_Sub;

        // PUBLISHERS
        ros::Publisher StateData_Pub;
        ros::Publisher FlipData_Pub;
        ros::Publisher ImpactData_Pub;
        ros::Publisher MiscData_Pub;

        // MESSAGES
        crazyflie_msgs::CF_StateData StateData_msg;
        crazyflie_msgs::CF_FlipData FlipData_msg;
        crazyflie_msgs::CF_ImpactData ImpactData_msg;
        crazyflie_msgs::CF_MiscData MiscData_msg;

        bool OnceFlag_flip = false;

};


void CF_DataConverter::ctrlData_Callback(const crazyflie_msgs::CtrlData &ctrl_msg)
{

    // ===================
    //     FLIGHT DATA
    // ===================
    StateData_msg.header.stamp = ros::Time::now();

    // CARTESIAN SPACE DATA
    StateData_msg.Pose.position = ctrl_msg.Pose.position;
    StateData_msg.Pose.orientation = ctrl_msg.Pose.orientation;
    StateData_msg.Twist.linear = ctrl_msg.Twist.linear;
    StateData_msg.Twist.angular = ctrl_msg.Twist.angular;


    // PROCESS EULER ANGLES
    float quat[4] = {
        (float)ctrl_msg.Pose.orientation.x,
        (float)ctrl_msg.Pose.orientation.y,
        (float)ctrl_msg.Pose.orientation.z,
        (float)ctrl_msg.Pose.orientation.w
    };
    float eul[3];
    quat2euler(quat,eul);
    StateData_msg.Eul.x = eul[0]*180/M_PI;
    StateData_msg.Eul.y = eul[1]*180/M_PI;
    StateData_msg.Eul.z = eul[2]*180/M_PI;


    // OPTICAL FLOW
    StateData_msg.Tau = ctrl_msg.Tau;
    StateData_msg.OFx = ctrl_msg.OFx;
    StateData_msg.OFy = ctrl_msg.OFy;
    StateData_msg.RREV = ctrl_msg.RREV;
    StateData_msg.D_ceil = ctrl_msg.D_ceil;

    // STATE SETPOINTS
    StateData_msg.x_d = ctrl_msg.x_d;
    StateData_msg.v_d = ctrl_msg.v_d;
    StateData_msg.a_d = ctrl_msg.a_d;

    // CONTROL ACTIONS
    StateData_msg.FM = ctrl_msg.FM;
    StateData_msg.MS_PWM = ctrl_msg.MS_PWM;

    // NEURAL NETWORK DATA
    StateData_msg.NN_flip = ctrl_msg.NN_flip;
    StateData_msg.NN_policy = ctrl_msg.NN_policy;


    // PUBLISH STATE DATA RECEIVED FROM CRAZYFLIE CONTROLLER
    StateData_Pub.publish(StateData_msg);


    // =================
    //     FLIP DATA
    // =================

    // CARTESIAN SPACE DATA
    if(ctrl_msg.flip_flag == true && OnceFlag_flip == false)
    {   
        FlipData_msg.header.stamp = ros::Time::now();
        OnceFlag_flip = true;

    }
    

    FlipData_msg.flip_flag = ctrl_msg.flip_flag;
    FlipData_msg.Pose_tr.position = ctrl_msg.Pose_tr.position;
    FlipData_msg.Pose_tr.orientation = ctrl_msg.Pose_tr.orientation;
    FlipData_msg.Twist_tr.linear = ctrl_msg.Twist_tr.linear;
    FlipData_msg.Twist_tr.angular = ctrl_msg.Twist_tr.angular;

    // OPTICAL FLOW
    FlipData_msg.Tau_tr = ctrl_msg.Tau_tr;
    FlipData_msg.OFx_tr = ctrl_msg.OFx_tr;
    FlipData_msg.OFy_tr = ctrl_msg.OFy_tr;
    FlipData_msg.RREV_tr = ctrl_msg.RREV_tr;
    FlipData_msg.D_ceil_tr = ctrl_msg.D_ceil_tr;

    // CONTROLLER ACTIONS
    FlipData_msg.FM_tr = ctrl_msg.FM_flip;

    // NEURAL NETWORK DATA
    FlipData_msg.NN_tr_flip = ctrl_msg.NN_tr_flip;
    FlipData_msg.NN_tr_policy = ctrl_msg.NN_tr_policy;

    // PUBLISH STATE DATA RECEIVED FROM CRAZYFLIE CONTROLLER
    FlipData_Pub.publish(FlipData_msg);

}

void CF_DataConverter::RL_CMD_Callback(const crazyflie_msgs::RLCmd::ConstPtr &msg)
{
    
    if(msg->cmd_type == 0)
    {
        FlipData_msg.header.stamp.sec = 0.0;
        FlipData_msg.header.stamp.nsec = 0.0;
        OnceFlag_flip = false;
    }


}

// void CF_DataConverter::log2_Callback(const crazyflie_msgs_exp::GenericLogData::ConstPtr &log2_msg)
// {
//     // // ANGULAR VELOCITY (Z)
//     // StateData_msg.Twist.angular.z = log2_msg->values[0]*1e-3;

//     // // CEILING DISTANCE
//     // StateData_msg.D_ceil = log2_msg->values[1]*1e-3;

//     // // DECOMPRESS THRUST/MOMENT MOTOR VALUES [g]
//     // float FM_z[2];
//     // float M_xy[2];

//     // decompressXY(log2_msg->values[2],FM_z);
//     // decompressXY(log2_msg->values[3],M_xy); 

//     // StateData_msg.FM = {FM_z[0],M_xy[0],M_xy[1],FM_z[1]}; // [F,Mx,My,Mz]


//     // // MOTOR PWM VALUES
//     // float MS_PWM12[2];
//     // float MS_PWM34[2];

//     // decompressXY(log2_msg->values[4],MS_PWM12);
//     // decompressXY(log2_msg->values[5],MS_PWM34);

//     // StateData_msg.MS_PWM = {
//     //     round(MS_PWM12[0]*2.0e3),
//     //     round(MS_PWM12[1]*2.0e3), 
//     //     round(MS_PWM34[0]*2.0e3),
//     //     round(MS_PWM34[1]*2.0e3)
//     // };
    
//     // // NEURAL NETWORK VALUES
//     // float NN_FP[2];
//     // decompressXY(log2_msg->values[6],NN_FP);
//     // StateData_msg.NN_flip = NN_FP[0];
//     // StateData_msg.NN_policy = NN_FP[1];


//     // // OTHER MISC INFO
//     // MiscData_msg.flip_flag = log2_msg->values[7];

//     // // PUBLISH MISC DATA RECEIVED FROM CRAZYFLIE (VIA CRAZYSWARM)
//     // MiscData_Pub.publish(MiscData_msg);
// }

// void CF_DataConverter::log3_Callback(const crazyflie_msgs_exp::GenericLogData::ConstPtr &log3_msg)
// {
//     // // POSITION SETPOINTS
//     // float xd_xy[2];
//     // decompressXY(log3_msg->values[0],xd_xy);

//     // StateData_msg.x_d.x = xd_xy[0];
//     // StateData_msg.x_d.y = xd_xy[1];
//     // StateData_msg.x_d.z = log3_msg->values[1]*1e-3;
   
//     // // VELOCITY SETPOINTS
//     // float vd_xy[2];
//     // decompressXY(log3_msg->values[2],vd_xy);

//     // StateData_msg.v_d.x = vd_xy[0];
//     // StateData_msg.v_d.y = vd_xy[1];
//     // StateData_msg.v_d.z = log3_msg->values[3]*1e-3;

//     // // ACCELERATION SETPOINTS
//     // float ad_xy[2];
//     // decompressXY(log3_msg->values[2],ad_xy);

//     // StateData_msg.a_d.x = ad_xy[0];
//     // StateData_msg.a_d.y = ad_xy[1];
//     // StateData_msg.a_d.z = log3_msg->values[5]*1e-3;

//     // MiscData_msg.battery_voltage = log3_msg->values[6]*1e-3;
// }

// void CF_DataConverter::log4_Callback(const crazyflie_msgs_exp::GenericLogData::ConstPtr &log4_msg)
// {
//     // // FLIP TRIGGER - POSITION
//     // FlipData_msg.Pose_tr.position.z = log4_msg->values[0]*1e-3;

//     // // FLIP TRIGGER - CEILING DISTANCE
//     // FlipData_msg.D_ceil = log4_msg->values[1]*1e-3;

//     // // FLIP TRIGGER - VELOCITY
//     // float vxy_arr[2];
//     // decompressXY(log4_msg->values[2],vxy_arr);
    
//     // FlipData_msg.Twist_tr.linear.x = vxy_arr[0];
//     // FlipData_msg.Twist_tr.linear.y = vxy_arr[1];
//     // FlipData_msg.Twist_tr.linear.z = log4_msg->values[3]*1e-3;

//     // // FLIP TRIGGER - ORIENTATION
//     // float quat_tr[4];
//     // uint32_t quatZ = (uint32_t)log4_msg->values[4];
//     // quatdecompress(quatZ,quat_tr);

//     // FlipData_msg.Pose_tr.orientation.x = quat_tr[0];
//     // FlipData_msg.Pose_tr.orientation.y = quat_tr[1];
//     // FlipData_msg.Pose_tr.orientation.z = quat_tr[2];
//     // FlipData_msg.Pose_tr.orientation.w = quat_tr[3]; 

//     // // FLIP TRIGGER - ANGULAR VELOCITY
//     // float wxy_arr[2];
//     // decompressXY(log4_msg->values[5],wxy_arr);
    
//     // FlipData_msg.Twist_tr.angular.x = wxy_arr[0];
//     // FlipData_msg.Twist_tr.angular.y = wxy_arr[1];
//     // FlipData_msg.Twist_tr.angular.z = log4_msg->values[5]*1e-3;

//     // // FLIP TRIGGER - OPTICAL FLOW
//     // float OF_xy_arr[2];
//     // decompressXY(log4_msg->values[6],OF_xy_arr);
    
//     // FlipData_msg.OFx_tr = OF_xy_arr[0];
//     // FlipData_msg.OFy_tr = OF_xy_arr[1];
//     // FlipData_msg.RREV_tr = log4_msg->values[7]*1e-3;

//     // // PUBLISH FLIP/IMPACT DATA
//     // FlipData_Pub.publish(FlipData_msg);   
//     // ImpactData_Pub.publish(ImpactData_msg);   

// }


// DECOMPRESS COMBINED PAIR OF VALUES FROM CF MESSAGE INTO THEIR RESPECTIVE FLOAT VALUES
void CF_DataConverter::decompressXY(uint32_t xy, float xy_arr[])
{
    uint16_t xd = ((uint32_t)xy >> 16) & 0xFFFF;    // Shift out y-value bits
    xy_arr[0] = ((float)xd - 32767.0f)*1e-3;        // Convert to normal value

    uint16_t yd = (uint32_t)xy & 0xFFFF;            // Save only y-value bits
    xy_arr[1] = ((float)yd - 32767.0f)*1e-3;

}

// CONVERT QUATERNION TO EULER ANGLES (YZX NOTATION)
void CF_DataConverter::quat2euler(float quat[], float eul[]){

    float R11,R21,R31,R22,R23;
    float phi,theta,psi; // (phi=>x,theta=>y,psi=>z)

    // CALC NEEDED ROTATION MATRIX COMPONENTS FROM QUATERNION
    R11 = 1.0 - 2.0*(pow(quat[1],2) + pow(quat[2],2) );
    R21 = 2.0*(quat[0]*quat[1] + quat[2]*quat[3]);
    R31 = 2.0*(quat[0]*quat[2] - quat[1]*quat[3]);

    R22 = 1.0 - 2.0*( pow(quat[0],2) + pow(quat[2],2) );
    R23 = 2.0*(quat[1]*quat[2] - quat[0]*quat[3]);

    // CONVERT ROTATION MATRIX COMPONENTS TO EULER ANGLES (YZX NOTATION)
    phi = atan2(-R23, R22);
    theta = atan2(-R31, R11);
    psi = asin(R21);

    eul[0] = phi;   // X-axis
    eul[1] = theta; // Y-axis
    eul[2] = psi;   // Z-axis
}