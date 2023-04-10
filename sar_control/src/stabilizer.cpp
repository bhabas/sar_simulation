#include "stabilizer.h"


void Controller::stabilizerLoop() // MAIN CONTROLLER LOOP
{
    ros::Rate rate(50);
    

    // RUN STABILIZER LOOP
    while(ros::ok)
    {
        printf("hello\n");
    

        tick++;
        rate.sleep();
    }
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "Controller_Node");
    ros::NodeHandle nh;

    Controller CTRL = Controller(&nh);
    ros::spin();

    
    return 0;
}

