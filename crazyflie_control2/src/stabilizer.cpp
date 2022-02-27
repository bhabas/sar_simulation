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
    
        if (RATE_DO_EXECUTE(100, tick)) {
            printf("t: %.4f \tCmd: \n",0.0);
            printf("Model: %s\n","CF_*****");
            printf("\n");

            printf("==== Flags ====\n");
            printf("Policy_armed:\t %u  Slowdown_type:\t %u  kp_xf:\t %u \n",policy_armed_flag,5,(int)kp_xf);
            printf("Flip_flag:\t %u  Impact_flag:\t %u  kd_xf:\t %u \n",flip_flag,5,(int)kd_xf);
            printf("Tumbled: \t %u  Tumble Detect:\t %u  Traj Active: %u \n",tumbled,tumble_detection,execute_traj);
            printf("Motorstop \t %u\n",motorstop_flag);
            printf("\n");

            printf("==== Setpoints ====\n");
            printf("x_d: %.3f  %.3f  %.3f\n",x_d.x,x_d.y,x_d.z);
            printf("v_d: %.3f  %.3f  %.3f\n",v_d.x,v_d.y,v_d.z);
            printf("a_d: %.3f  %.3f  %.3f\n",a_d.x,a_d.y,a_d.z);
            printf("\n");

            printf("==== System States ====\n");
            printf("Pos [m]:\t %.3f  %.3f  %.3f\n",statePos.x,statePos.y,statePos.z);
            printf("Vel [m/s]:\t %.3f  %.3f  %.3f\n",stateVel.x,stateVel.y,stateVel.z);
            printf("Omega [rad/s]:\t %.3f  %.3f  %.3f\n",stateOmega.x,stateOmega.y,stateOmega.z);
            printf("Eul [deg]:\t %.3f  %.3f  %.3f\n",stateEul.x,stateEul.y,stateEul.z);
            printf("\n");

            printf("Tau: %.3f \tOFx: %.3f \tOFy: %.3f \tRREV: %.3f\n",Tau,OFx,OFy,RREV);
            printf("D_ceil: %.3f\n",d_ceil);
            printf("\n");

            printf("==== Policy Values ====\n");
            printf("RL: \n");
            printf("RREV_thr: %.3f \tG1: %.3f \tG2: %.3f\n",RREV_thr,G1,G2);
            printf("\n");

            printf("NN_Outputs: \n");
            printf("NN_Flip:  %.3f \tNN_Policy: %.3f \n",NN_flip,NN_policy);
            printf("\n");

            printf("==== Flip Trigger Values ====\n");
            printf("RREV_tr:    %.3f \tNN_tr_Flip:    %.3f \n",RREV_tr,NN_tr_flip);
            printf("OFy_tr:     %.3f \tNN_tr_Policy:  %.3f \n",OFy_tr,NN_tr_policy);
            printf("D_ceil_tr:  %.3f \n",d_ceil_tr);
            printf("\n");

            printf("==== Controller Actions ====\n");
            printf("FM [N/N*mm]: %.3f  %.3f  %.3f  %.3f\n",F_thrust,M.x*1.0e3,M.y*1.0e3,M.z*1.0e3);
            printf("f [g]: %.3f  %.3f  %.3f  %.3f\n",f_thrust_g,f_roll_g,f_pitch_g,f_yaw_g);
            printf("\n");

            printf("MS_PWM: %u  %u  %u  %u\n",M1_pwm,M2_pwm,M3_pwm,M4_pwm);
            printf("\n");


            printf("=== Parameters ====\n");
            printf("Kp_P: %.3f  %.3f  %.3f \t",Kp_p.x,Kp_p.y,Kp_p.z);
            printf("Kp_R: %.3f  %.3f  %.3f \n",Kp_R.x,Kp_R.y,Kp_R.z);
            printf("Kd_P: %.3f  %.3f  %.3f \t",Kd_p.x,Kd_p.y,Kd_p.z);
            printf("Kd_R: %.3f  %.3f  %.3f \n",Kd_R.x,Kd_R.y,Kd_R.z);
            printf("Ki_P: %.3f  %.3f  %.3f \t",Ki_p.x,Ki_p.y,Ki_p.z);
            printf("Ki_R: %.3f  %.3f  %.3f \n",Ki_p.x,Ki_p.y,Ki_p.z);
            printf("======\n");


        }


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

    StateCollector SC = StateCollector(&nh);
    ros::spin();

    
    return 0;
}

