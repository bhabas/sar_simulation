#include "stabilizer.h"
#include "controller_gtc.h"

void Controller::stabilizerLoop() // MAIN CONTROLLER LOOP
{
    ros::Rate rate(1000);
    
    // TURN OFF SAFEMODE (EXPERIMENT ONLY SETTING)
    safeModeEnable = false;

    // INITIATE CONTROLLER
    controllerGTCInit();

    // RUN STABILIZER LOOP
    while(ros::ok)
    {
        stateEstimator(&state, &sensorData, &control, tick); // Run state/sensor values through "Kalman filter"
        controllerGTC(&control, &setpoint, &sensorData, &state, tick);
    

        Controller::publishCtrlData();
        Controller::publishCtrlDebug();


        // PUBLISH MOTOR COMMANDS
        MS_PWM_msg.MotorPWM = {M1_pwm,M2_pwm,M3_pwm,M4_pwm};
        CF_PWM_Publisher.publish(MS_PWM_msg);

        
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

