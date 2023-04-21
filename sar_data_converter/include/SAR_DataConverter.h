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

// CUSTOM INCLUDES


class SAR_DC {

    public:

        SAR_DC(ros::NodeHandle* nh)
        {
            method_b();

        }

        void MainInit();
        void MainLoop();
        void method_b();
        
        void ConsoleLoop();

    
};

