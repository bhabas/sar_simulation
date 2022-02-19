#include "stabilizer_types.h"

// ROS Includes
#include <ros/ros.h>

uint32_t tick;

int main(int argc, char** argv)
{   
    ros::Time::init();
    tick = 1;
    ros::Rate rate(RATE_MAIN_LOOP);
    while(ros::ok)
    {

        printf("New test\n");
        tick++;
        rate.sleep();
    }



}