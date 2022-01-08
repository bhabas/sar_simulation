#include <iostream>

#include "controller.h"

#include <ros/ros.h>



void controllerGTC(state_t *state, const uint32_t tick)
{
    // printf("HELLO\n");
    
    printf("%.5f\n",state->velocity.x);


    
}

int main(int argc, char **argv)
{   
    ros::init(argc, argv,"controller_node");
    ros::NodeHandle nh;
    Controller controller = Controller(&nh);
    // controller.controllerGTCReset();
    // controller.startController();
    ros::spin();
}