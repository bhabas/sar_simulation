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


        // ===================
        //     FLIGHT DATA
        // ===================

        ros::Time Time;
        std_msgs::Header header;

        geometry_msgs::Pose Pose;
        geometry_msgs::Twist Twist;
        geometry_msgs::Vector3 Eul;

        double Tau;
        double OFx;
        double OFy;
        double RREV;
        double D_ceil;

        boost::array<double, 4> FM;
        boost::array<double, 4> MS_PWM;

        double NN_flip;
        double NN_policy;

        geometry_msgs::Vector3 x_d;
        geometry_msgs::Vector3 v_d;
        geometry_msgs::Vector3 a_d;

        // ==================
        //     FLIP DATA
        // ==================

        bool flip_flag = false;
        bool OnceFlag_flip = false;

        ros::Time Time_flip;
        std_msgs::Header header_flip;

        geometry_msgs::Pose Pose_tr;
        geometry_msgs::Twist Twist_tr;

        double Tau_tr;
        double OFx_tr;
        double OFy_tr;
        double RREV_tr;
        double D_ceil_tr;

        boost::array<double, 4> FM_tr;

        double NN_tr_flip;
        double NN_tr_policy;


        // ===================
        //     IMPACT DATA
        // ===================

        bool impact_flag = false;
        bool OnceFlag_impact = false;

        ros::Time Time_impact;
        std_msgs::Header header_impact;

        geometry_msgs::Pose Pose_impact;
        geometry_msgs::Twist Twist_impact;
        geometry_msgs::Vector3 Force_impact;
        uint16_t Pad_Connections = 0;


        // ==================
        //     MISC DATA
        // ==================

        std_msgs::Header header_misc;
        double V_battery = 0.0;


        


};

void CF_DataConverter::ctrlData_Callback(const crazyflie_msgs::CtrlData &ctrl_msg)
{
    // ===================
    //     FLIGHT DATA
    // ===================
    Time = ros::Time::now();
    Pose = ctrl_msg.Pose;
    Twist = ctrl_msg.Twist;

    // PROCESS EULER ANGLES
    float quat[4] = {
        (float)ctrl_msg.Pose.orientation.x,
        (float)ctrl_msg.Pose.orientation.y,
        (float)ctrl_msg.Pose.orientation.z,
        (float)ctrl_msg.Pose.orientation.w
    };
    float eul[3];
    quat2euler(quat,eul);
    Eul.x = eul[0]*180/M_PI;
    Eul.y = eul[1]*180/M_PI;
    Eul.z = eul[2]*180/M_PI;

    // OPTICAL FLOW
    Tau = ctrl_msg.Tau;
    OFx = ctrl_msg.OFx;
    OFy = ctrl_msg.OFy;
    RREV = ctrl_msg.RREV;
    D_ceil = ctrl_msg.D_ceil;

    // STATE SETPOINTS
    x_d = ctrl_msg.x_d;
    v_d = ctrl_msg.v_d;
    a_d = ctrl_msg.a_d;

    // CONTROL ACTIONS
    FM = ctrl_msg.FM;
    MS_PWM = ctrl_msg.MS_PWM;

    // NEURAL NETWORK DATA
    NN_flip = ctrl_msg.NN_flip;
    NN_policy = ctrl_msg.NN_policy;


    // =================
    //     FLIP DATA
    // =================

    // CARTESIAN SPACE DATA
    if(ctrl_msg.flip_flag == true && OnceFlag_flip == false)
    {   
        // FlipData_msg.header.stamp = ros::Time::now();
        OnceFlag_flip = true;

    }
    

    flip_flag = ctrl_msg.flip_flag;
    Pose_tr = ctrl_msg.Pose_tr;
    Twist_tr = ctrl_msg.Twist_tr;

    // OPTICAL FLOW
    Tau_tr = ctrl_msg.Tau_tr;
    OFx_tr = ctrl_msg.OFx_tr;
    OFy_tr = ctrl_msg.OFy_tr;
    RREV_tr = ctrl_msg.RREV_tr;
    D_ceil_tr = ctrl_msg.D_ceil_tr;

    // CONTROLLER ACTIONS
    FM_tr = ctrl_msg.FM_flip;

    // NEURAL NETWORK DATA
    NN_tr_flip = ctrl_msg.NN_tr_flip;
    NN_tr_policy = ctrl_msg.NN_tr_policy;


}



//     // PUBLISH STATE DATA RECEIVED FROM CRAZYFLIE CONTROLLER
//     StateData_Pub.publish(StateData_msg);


//     // =================
//     //     FLIP DATA
//     // =================

//     // CARTESIAN SPACE DATA
//     if(ctrl_msg.flip_flag == true && OnceFlag_flip == false)
//     {   
//         FlipData_msg.header.stamp = ros::Time::now();
//         OnceFlag_flip = true;

//     }
    

//     FlipData_msg.flip_flag = ctrl_msg.flip_flag;
//     FlipData_msg.Pose_tr.position = ctrl_msg.Pose_tr.position;
//     FlipData_msg.Pose_tr.orientation = ctrl_msg.Pose_tr.orientation;
//     FlipData_msg.Twist_tr.linear = ctrl_msg.Twist_tr.linear;
//     FlipData_msg.Twist_tr.angular = ctrl_msg.Twist_tr.angular;

//     // OPTICAL FLOW
//     FlipData_msg.Tau_tr = ctrl_msg.Tau_tr;
//     FlipData_msg.OFx_tr = ctrl_msg.OFx_tr;
//     FlipData_msg.OFy_tr = ctrl_msg.OFy_tr;
//     FlipData_msg.RREV_tr = ctrl_msg.RREV_tr;
//     FlipData_msg.D_ceil_tr = ctrl_msg.D_ceil_tr;

//     // CONTROLLER ACTIONS
//     FlipData_msg.FM_tr = ctrl_msg.FM_flip;

//     // NEURAL NETWORK DATA
//     FlipData_msg.NN_tr_flip = ctrl_msg.NN_tr_flip;
//     FlipData_msg.NN_tr_policy = ctrl_msg.NN_tr_policy;

//     // PUBLISH STATE DATA RECEIVED FROM CRAZYFLIE CONTROLLER
//     FlipData_Pub.publish(FlipData_msg);

// }

void CF_DataConverter::RL_CMD_Callback(const crazyflie_msgs::RLCmd::ConstPtr &msg)
{
    
    if(msg->cmd_type == 0)
    {
        FlipData_msg.header.stamp.sec = 0.0;
        FlipData_msg.header.stamp.nsec = 0.0;
        OnceFlag_flip = false;
        OnceFlag_impact = false;
    }


}


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