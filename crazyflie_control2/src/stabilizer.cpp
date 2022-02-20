#include "stabilizer.h"


void StateCollector::stabilizerLoop() // MAIN CONTROLLER LOOP
{
    ros::Rate rate(1000);
    tick = 1;

    // INITIATE CONTROLLER
    controllerGTCInit();
   
   // RUN STABILIZER LOOP
    while(ros::ok)
    {

        stateEstimator(&state, &sensorData, &control, tick); // Run state/sensor values through "Kalman filter"
        controllerGTC(&control, &setpoint, &sensorData, &state, tick);


        
        tick++;
        rate.sleep();
    }
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "Stabilizer_Node");
    ros::NodeHandle nh;

    StateCollector SC = StateCollector(&nh);
    ros::spin();

    
    return 0;
}

