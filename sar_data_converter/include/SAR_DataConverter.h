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

#include <gazebo_msgs/SetPhysicsProperties.h>

// CUSTOM INCLUDES


class SAR_DataConverter {

    public:

        SAR_DataConverter(ros::NodeHandle* nh)
        {

            GZ_SimSpeed_Client = nh->serviceClient<gazebo_msgs::SetPhysicsProperties>("/gazebo/set_physics_properties");


            SAR_DC_Thread = std::thread(&SAR_DataConverter::MainLoop, this);

        }

        void MainInit();
        void MainLoop();


        void adjustSimSpeed(float speed_mult);

        
        void ConsoleLoop();

    private:

        uint32_t tick = 0;      // Tick for each loop iteration
        std::thread SAR_DC_Thread;
        std::thread ConsoleOutput_Thread;
        ros::Time Time_start;   // Initial time in UNIX notation


        ros::ServiceClient GZ_SimSpeed_Client;
    
};

