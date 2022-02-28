#include "stabilizer.h"
#include "controller_gtc.h"

void Controller::stabilizerLoop() // MAIN CONTROLLER LOOP
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

        t = ros::Time::now();
    
        if (RATE_DO_EXECUTE(20, tick)) {
            // Controller::consoleOuput();
        }

        Controller::checkSlowdown();
        


        MS_msg.MotorPWM = {M1_pwm,M2_pwm,M3_pwm,M4_pwm};
        MS_PWM_Publisher.publish(MS_msg);

        
        tick++;
        rate.sleep();
    }
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "Stabilizer_Node");
    ros::NodeHandle nh;

    Controller SC = Controller(&nh);
    ros::spin();

    
    return 0;
}

