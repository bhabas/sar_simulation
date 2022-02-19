#include "stabilizer_types.h"
#include "controller_gtc.h"

// STANDARD LIBRARIES
#include <math.h>       
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <pwd.h>

// ROS Includes
#include <ros/ros.h>
#include "crazyflie_msgs/CtrlData.h"
#include "crazyflie_msgs/ImpactData.h"
#include "crazyflie_msgs/RLCmd.h"
#include "crazyflie_msgs/RLData.h"
#include "crazyflie_msgs/PadConnect.h"
#include "crazyflie_msgs/MS.h"
uint32_t tick;

// State variables for the stabilizer
static setpoint_t setpoint;
static sensorData_t sensorData;
static state_t state;
static control_t control;

int main(int argc, char **argv)
{
    tick = 1;
    ros::Rate rate(RATE_MAIN_LOOP);

    while(ros::ok)
    {
        // stateEstimator(&state, &sensorData, &control, tick);
        // compressState();

        // commanderGetSetpoint(&setpoint, &state);
        // compressSetpoint();

        // sitAwUpdateSetpoint(&setpoint, &sensorData, &state); // Situation Awareness (Tumble,Freefall,At Rest)
        // collisionAvoidanceUpdateSetpoint(&setpoint, &sensorData, &state, tick);

        // controller(&control, &setpoint, &sensorData, &state, tick);
        controllerGTC(&control, &setpoint, &sensorData, &state, tick);


        tick++;
        rate.sleep();
    }

    return 1;
}