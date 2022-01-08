#include <iostream>

#include "controller.h"

#include <ros/ros.h>



void Controller::controllerGTC()
{

    ros::Rate rate(500);
    unsigned int t_step = 0; // t_step counter
       
    while(ros::ok)
    {
        
        cout << "Hello" << endl;
        rate.sleep();
    }
}


int main(int argc, char **argv)
{   
    ros::init(argc, argv,"controller_node");
    ros::NodeHandle nh;
    Controller controller = Controller(&nh);
    // controller.controllerGTCReset();
    controller.Load();
    ros::spin();
}