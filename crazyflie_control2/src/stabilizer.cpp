#include "stabilizer.h"
#include "stabilizer_types.h"
#include "nml.h"
#include "controller_gtc.h"
#include <ros/ros.h>


uint32_t tick;

int main(int argc, char **argv)
{

    ros::init(argc, argv, "Stabilizer_Node");
    ros::NodeHandle nh;

    tick = 1;
    ros::Rate rate(RATE_MAIN_LOOP);

    while(ros::ok)
    {
        printHello();
        rate.sleep();

        tick++;
    }
    return 0;
}