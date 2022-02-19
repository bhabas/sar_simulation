#include "stabilizer.h"
#include "stabilizer_types.h"
#include "nml.h"


#include "controller_gtc.h"
#include "estimator.h"
#include <ros/ros.h>


uint32_t tick;

// State variables for the stabilizer
static setpoint_t setpoint;
static sensorData_t sensorData;
static state_t state;
static control_t control;

int main(int argc, char **argv)
{

    ros::init(argc, argv, "Stabilizer_Node");
    ros::NodeHandle nh;

    tick = 1;
    ros::Rate rate(RATE_MAIN_LOOP);

    while(ros::ok)
    {
        stateEstimator(&state, &sensorData, &control, tick);
        controllerGTC(&control, &setpoint, &sensorData, &state, tick);

        tick++;
        rate.sleep();
    }
    return 0;
}