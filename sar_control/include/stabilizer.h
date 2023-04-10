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


// STANDARD Includes
#include <iostream>
#include <thread>
#include <stdio.h>
#include <string>

// FIRMWARE INCLUDES


// ROS INCLUDES
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include "sensor_msgs/Image.h"


#include "crazyflie_msgs/OF_SensorData.h"
#include "crazyflie_msgs/MS.h"

#include "crazyflie_msgs/CtrlData.h"
#include "crazyflie_msgs/CtrlDebug.h"

#include "crazyflie_msgs/GTC_Cmd_srv.h"
#include "crazyflie_msgs/RLData.h"


class Controller
{
    public:

        Controller(ros::NodeHandle *nh)
        {
            // Thread main controller loop so other callbacks can work fine
            controllerThread = std::thread(&Controller::stabilizerLoop, this);
        }

    

        // DEFINE THREAD OBJECTS
        std::thread controllerThread;
        std::thread appThread;

        uint32_t tick = 1;

        void stabilizerLoop();
        void appLoop();

};
