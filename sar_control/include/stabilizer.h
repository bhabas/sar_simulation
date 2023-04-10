#pragma once

// STANDARD Includes
#include <iostream>
#include <thread>
#include <stdio.h>
#include <string>

// FIRMWARE INCLUDES
#include "app.h"

// ROS INCLUDES
#include <ros/ros.h>


class Controller
{
    public:

        Controller(ros::NodeHandle *nh)
        {
            // Thread main controller loop so other callbacks can work fine
            appThread = std::thread(&Controller::appLoop, this);
        }

    

        // DEFINE THREAD OBJECTS
        std::thread appThread;

        uint32_t tick = 1;

        void appLoop();

};
