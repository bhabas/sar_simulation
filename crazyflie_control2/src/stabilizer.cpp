#include "stabilizer.h"
#include "controller_gtc.h"

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

        printf("val: %u\n",val);
    


        // MS_msg.MotorPWM = {M1_pwm,M2_pwm,M3_pwm,M4_pwm};
        // MS_PWM_Publisher.publish(MS_msg);

        
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

