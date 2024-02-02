#include "stabilizer.h"

void Controller::appLoop()
{
    ros::Rate rate(1000);

    // RUN STABILIZER LOOP
    while(ros::ok)
    {
        appMain();
        
        rate.sleep();
    }
}

void Controller::stabilizerLoop() // MAIN CONTROLLER LOOP
{
    ros::Rate rate(1000);
    loadInitParams();
    

    // INITIATE CONTROLLER
    controllerOutOfTreeInit();

    // RUN STABILIZER LOOP
    while(ros::ok)
    {
        printf("Tick: %d\n",tick);
        controllerOutOfTree(&control, &setpoint, &sensorData, &state, tick);
    

        Controller::publishCtrlData();
        Controller::publishCtrlDebug();

        rate.sleep(); // Process holds here till next tick
        tick++;

    }
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "SAR_Controller_Node");
    ros::NodeHandle nh;

    Controller CTRL = Controller(&nh);
    ros::spin();

    
    return 0;
}

