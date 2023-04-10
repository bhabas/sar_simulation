#include "stabilizer.h"

void Controller::appLoop()
{
    ros::Rate rate(100);

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
    

    // INITIATE CONTROLLER
    controllerOutOfTreeInit();

    // RUN STABILIZER LOOP
    while(ros::ok)
    {
        // stateEstimator(&state, &sensorData, &control, tick); // Run state/sensor values through "Kalman filter"
        controllerOutOfTree(&control, &setpoint, &sensorData, &state, tick);
        // printf("Stab loop\n");
    

        // Controller::publishCtrlData();
        // Controller::publishCtrlDebug();


        
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

