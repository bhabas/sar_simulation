/* 
This script subscribes to the ROS topics with the raw data from the crazyflie,
organizes it, and then republishes the data in an organized manner that 
is easy to use.
*/

#include <stdio.h>
#include <iostream>
#include <boost/circular_buffer.hpp>

#include <ros/ros.h>
#include <math.h>       /* sqrt */
#include <geometry_msgs/WrenchStamped.h>
#include "quatcompress.h"

// MESSAGE INCLUDES
#include "crazyflie_msgs/CF_StateData.h"
#include "crazyflie_msgs/CF_FlipData.h"
#include "crazyflie_msgs/CF_ImpactData.h"
#include "crazyflie_msgs/CF_MiscData.h"

#include "crazyflie_msgs/CtrlData.h"
#include "crazyflie_msgs/RLCmd.h"


class CF_DataConverter
{
    public:
        // CONSTRUCTOR TO START PUBLISHERS AND SUBSCRIBERS (Similar to Python's __init__())
        CF_DataConverter(ros::NodeHandle* nh)
        {
            // INITIALIZE SUBSCRIBERS
            CTRL_Data_Subscriber = nh->subscribe("/CTRL/data", 1, &CF_DataConverter::CtrlData_Callback, this, ros::TransportHints().tcpNoDelay());
            RL_CMD_Subscriber = nh->subscribe("/RL/cmd",5,&CF_DataConverter::RL_CMD_Callback,this,ros::TransportHints().tcpNoDelay());
            Surface_FT_Subscriber = nh->subscribe("/ENV/Surface_FT_sensor",5,&CF_DataConverter::SurfaceFT_Sensor_Callback,this,ros::TransportHints().tcpNoDelay());

            // INITIALIZE PUBLISHERS
            StateData_Pub = nh->advertise<crazyflie_msgs::CF_StateData>("/CF_DC/StateData",1);
            MiscData_Pub =  nh->advertise<crazyflie_msgs::CF_MiscData>("/CF_DC/MiscData",1);
            FlipData_Pub =  nh->advertise<crazyflie_msgs::CF_FlipData>("/CF_DC/FlipData",1);
            ImpactData_Pub = nh->advertise<crazyflie_msgs::CF_ImpactData>("/CF_DC/ImpactData",1);         



        }


        // FUNCTION PRIMITIVES
        void CtrlData_Callback(const crazyflie_msgs::CtrlData &ctrl_msg);
        void RL_CMD_Callback(const crazyflie_msgs::RLCmd::ConstPtr &msg);
        void SurfaceFT_Sensor_Callback(const geometry_msgs::WrenchStamped::ConstPtr &msg);

        void Publish_StateData();
        void Publish_FlipData();
        void Publish_ImpactData();
        void Publish_MiscData();


        void decompressXY(uint32_t xy, float xy_arr[]);
        void quat2euler(float quat[], float eul[]);

    private:

        // SUBSCRIBERS
        ros::Subscriber CTRL_Data_Subscriber;
        ros::Subscriber RL_CMD_Subscriber;
        ros::Subscriber Surface_FT_Subscriber;

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

        ros::Time Time_tr;

        geometry_msgs::Pose Pose_tr;
        geometry_msgs::Twist Twist_tr;
        geometry_msgs::Vector3 Eul_tr;


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
        geometry_msgs::Vector3 Force_impact;
        geometry_msgs::Pose Pose_impact;
        geometry_msgs::Twist Twist_impact;
        geometry_msgs::Vector3 Eul_impact;


        double impact_force_x = 0.0; // Max impact force in X-direction [N]
        double impact_force_y = 0.0; // Max impact force in Y-direction [N]
        double impact_force_z = 0.0; // Max impact force in Z-direction [N]
        double impact_force_resultant = 0.0; // 

        boost::circular_buffer<geometry_msgs::Pose> Pose_impact_buff {5};
        boost::circular_buffer<geometry_msgs::Twist> Twist_impact_buff {5};
        uint16_t Pad_Connections = 0;

        


        // ==================
        //     MISC DATA
        // ==================

        std_msgs::Header header_misc;
        double V_battery = 0.0;


        


};

void CF_DataConverter::SurfaceFT_Sensor_Callback(const geometry_msgs::WrenchStamped::ConstPtr &msg)
{
    // RECORD MAX FORCE EXPERIENCED
    if (msg->wrench.force.x > impact_force_x){
        impact_force_x = msg->wrench.force.x;
    }
    if (msg->wrench.force.y > impact_force_y){
        impact_force_y = msg->wrench.force.y;
    }
    if (msg->wrench.force.z > impact_force_z){
        impact_force_z = msg->wrench.force.z;
    }

    impact_force_resultant = sqrt(
        pow(msg->wrench.force.x,2) + 
        pow(msg->wrench.force.y,2) + 
        pow(msg->wrench.force.z,2));

    if (impact_force_resultant >= 0.1 && impact_flag == false){ 
        // LOCK IN STATE DATA WHEN IMPACT DETECTED
        impact_flag = true;

        // RECORD IMPACT STATE DATA FROM 2 DATAPOINTS BEHIND WHEN IMPACT FLAGGED
        Time_impact = ros::Time::now();
        Pose_impact = Pose_impact_buff.front();
        Twist_impact = Twist_impact_buff.front();

        // PROCESS EULER ANGLES
        float quat_impact[4] = {
            (float)Pose_impact.orientation.x,
            (float)Pose_impact.orientation.y,
            (float)Pose_impact.orientation.z,
            (float)Pose_impact.orientation.w
        };
        float eul_impact[3];
        quat2euler(quat_impact,eul_impact);
        Eul_impact.x = eul_impact[0]*180/M_PI;
        Eul_impact.y = eul_impact[1]*180/M_PI;
        Eul_impact.z = eul_impact[2]*180/M_PI;

    }



}



void CF_DataConverter::Publish_ImpactData()
{
    ImpactData_msg.impact_flag = impact_flag;
    ImpactData_msg.header.stamp = Time_impact;

    ImpactData_msg.Force_impact.x = impact_force_x;
    ImpactData_msg.Force_impact.y = impact_force_y;
    ImpactData_msg.Force_impact.z = impact_force_z;

    ImpactData_msg.Pose_impact = Pose_impact;
    ImpactData_msg.Twist_impact = Twist_impact;
    ImpactData_msg.Eul_impact = Eul_impact;



    ImpactData_Pub.publish(ImpactData_msg);
}
