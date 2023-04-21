#pragma once


// STANDARD INCLUDES
#include <stdio.h>
#include <iostream>
#include <boost/circular_buffer.hpp>
#include <math.h>       /* sqrt */
#include <thread>
#include <locale.h>

#include <ncurses.h>
#include <unistd.h>
#include <ctime>

// ROS INCLUDES
#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Point.h>

#include <gazebo_msgs/ContactsState.h>
#include <gazebo_msgs/SetPhysicsProperties.h>
#include <gazebo_msgs/SetModelState.h>

// CUSTOM INCLUDES
#include "crazyflie_msgs/CF_StateData.h"
#include "crazyflie_msgs/CF_FlipData.h"
#include "crazyflie_msgs/CF_ImpactData.h"
#include "crazyflie_msgs/CF_MiscData.h"

#include "crazyflie_msgs/CtrlData.h"
#include "crazyflie_msgs/CtrlDebug.h"
#include "crazyflie_msgs/GTC_Cmd_srv.h"
#include "crazyflie_msgs/GTC_Cmd.h"


#include "crazyflie_msgs/RLData.h"
#include "crazyflie_msgs/PadConnect.h"

#include "crazyflie_msgs/activateSticky.h"
#include "crazyflie_msgs/loggingCMD.h"
#include "crazyflie_msgs/GenericLogData.h"

#include "quatcompress.h"


class SAR_DataConverter {

    public:

        SAR_DataConverter(ros::NodeHandle* nh)
        {

            GZ_SimSpeed_Client = nh->serviceClient<gazebo_msgs::SetPhysicsProperties>("/gazebo/set_physics_properties");


            SAR_DC_Thread = std::thread(&SAR_DataConverter::MainLoop, this);

        }

        void MainInit();
        void MainLoop();
        void ConsoleLoop();


        // =======================
        //     GAZEBO FUNCTIONS
        // =======================
        void activateStickyFeet();
        void checkSlowdown();
        void adjustSimSpeed(float speed_mult);
        void Update_Landing_Surface_Pose(float Pos_x, float Pos_y, float Pos_z, float Plane_Angle);

        // =======================
        //     GAZEBO CALLBACKS
        // =======================
        void CtrlData_Callback(const crazyflie_msgs::CtrlData &ctrl_msg);
        void CtrlDebug_Callback(const crazyflie_msgs::CtrlDebug &ctrl_msg);

        void SurfaceFT_Sensor_Callback(const geometry_msgs::WrenchStamped::ConstPtr &msg);
        void Surface_Contact_Callback(const gazebo_msgs::ContactsState &msg);
        void Pad_Connections_Callback(const crazyflie_msgs::PadConnect &msg);


        // =================================
        //     EXPERIMENT DATA CALLBACKS
        // =================================
        void decompressXY(uint32_t xy, float xy_arr[]);
        void log1_Callback(const crazyflie_msgs::GenericLogData::ConstPtr &log1_msg);
        void log2_Callback(const crazyflie_msgs::GenericLogData::ConstPtr &log2_msg);
        void log3_Callback(const crazyflie_msgs::GenericLogData::ConstPtr &log3_msg);
        void log4_Callback(const crazyflie_msgs::GenericLogData::ConstPtr &log4_msg);
        void log5_Callback(const crazyflie_msgs::GenericLogData::ConstPtr &log5_msg);
        void log6_Callback(const crazyflie_msgs::GenericLogData::ConstPtr &log6_msg);


        // =======================
        //    LOGGING FUNCTIONS
        // =======================
        bool DataLogging_Callback(crazyflie_msgs::loggingCMD::Request &req, crazyflie_msgs::loggingCMD::Response &res);
        void create_CSV();
        void append_CSV_states();
        void append_CSV_misc();
        void append_CSV_flip();
        void append_CSV_impact();
        void append_CSV_blank();



        // =================================
        //     ORGANIZED DATA PUBLISHERS
        // =================================
        void Publish_StateData();
        void Publish_FlipData();
        void Publish_ImpactData();
        void Publish_MiscData();


        // =======================
        //     MISC. FUNCTIONS
        // =======================
        void quat2euler(float quat[], float eul[]);
        void euler2quat(float quat[],float eul[]);
        void LoadParams();


        

    private:

        uint32_t tick = 0;      // Tick for each loop iteration
        std::thread SAR_DC_Thread;
        std::thread ConsoleOutput_Thread;
        ros::Time Time_start;   // Initial time in UNIX notation


        ros::ServiceClient GZ_SimSpeed_Client;
    
};

