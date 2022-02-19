#include "stabilizer_types.h"
#include "nml.h"

#include <iostream>
#include <thread>
#include <cmath>        // std::abs


#include "controller_gtc.h"
#include "estimator.h"
#include <ros/ros.h>

#include <stdio.h>
#include <ros/ros.h>
#include "example_msgs/CustomMessage.h"
#include "nav_msgs/Odometry.h"



uint32_t tick;

// State variables for the stabilizer
static setpoint_t setpoint;
static sensorData_t sensorData;
static state_t state;
static control_t control;

class StateCollector
{
    public:

        StateCollector(ros::NodeHandle *nh)
        {
            sub = nh->subscribe("/env/vicon_state",1,&StateCollector::vicon_Callback,this,ros::TransportHints().tcpNoDelay());


            controllerThread = std::thread(&StateCollector::stabilizerLoop, this);
        }

        void vicon_Callback(const nav_msgs::Odometry::ConstPtr &msg);
        void stabilizerLoop();

        ros::Subscriber sub;

        // DEFINE THREAD OBJECTS
        std::thread controllerThread;

        uint32_t tick;


};


void StateCollector::vicon_Callback(const nav_msgs::Odometry::ConstPtr &msg)
{

    // std::cout << msg->pose.pose.position.x << std::endl;
    
    state.position.x = msg->pose.pose.position.x;


}


