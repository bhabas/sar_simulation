#include "stabilizer.h"

void Controller::appLoop()
{
    ros::Rate rate(50);

    // RUN STABILIZER LOOP
    while(ros::ok)
    {
        appMain();
        
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

