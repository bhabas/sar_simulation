#include "controller_GTC.h"
#include "led.h"


void appMain() {

    while (1)
    {
        #ifdef CONFIG_SAR_SIM

            if (GTC_Cmd.cmd_rx == true)
            {
                GTC_Command(&GTC_Cmd);
                GTC_Cmd.cmd_rx = false;
            }


        #elif CONFIG_SAR_EXP

            if (appchannelReceiveDataPacket(&GTC_Cmd,sizeof(GTC_Cmd),APPCHANNEL_WAIT_FOREVER))
            {
                if (GTC_Cmd.cmd_rx == true)
                {
                    GTC_Command(&GTC_Cmd);
                    GTC_Cmd.cmd_rx = false;
                }
            }

        #endif
    }
    
}

    


void controllerOutOfTreeInit() {
    consolePrintf("GTC init\n");

    ledSet(LED_BLUE_L, 0);
    ledSet(LED_BLUE_NRF, 0);

}

bool controllerOutOfTreeTest() {

  return true;
}

void controllerOutOfTreeReset() {

    consolePrintf("GTC Reset\n");

    // RESET INTEGRATION ERRORS
    e_PI = vzero();
    e_RI = vzero();

    // TURN POS/VEL CONTROLLER FLAGS ON
    kp_xf = 1.0f;
    kd_xf = 1.0f;

    // RESET SETPOINTS TO HOME POSITION
    x_d = mkvec(0.0f,0.0f,0.4f);
    v_d = mkvec(0.0f,0.0f,0.0f);
    a_d = mkvec(0.0f,0.0f,0.0f);
    b1_d = mkvec(1.0f,0.0f,0.0f);

    // RESET SYSTEM FLAGS
    tumbled = false;
    motorstop_flag = false;
    customThrust_flag = false;
    customPWM_flag = false;
    moment_flag = false;

    // RESET TRAJECTORY FLAGS


    // RESET POLICY FLAGS

}


void controllerOutOfTree(control_t *control,const setpoint_t *setpoint, 
                                            const sensorData_t *sensors, 
                                            const state_t *state, 
                                            const uint32_t tick) 
{

    if (RATE_DO_EXECUTE(2, tick))
    {
        // consolePrintf("GTC loop value1: %.3f\n",(double)GTC_Cmd.cmd_val1);
    }

    // UPDATE OPTICAL FLOW VALUES AT 100 HZ
    if (RATE_DO_EXECUTE(RATE_100_HZ, tick)) {

    }
    

    if (RATE_DO_EXECUTE(RATE_500_HZ, tick)) {

        controlOutput(state,sensors);

        if(moment_flag == true || flip_flag == true)
        {
            // Controller defaults to increase front motor & decrease back motors to flip
            // Instead double front motors and set back motors to zero for desired body moment
            // This gives same moment but avoids negative motor speeds
            F_thrust = 0.0f;
            M = vscl(2.0f,M_d);
        }

        // =========== CONVERT THRUSTS [N] AND MOMENTS [N*m] TO PWM =========== // 
        f_thrust_g = clamp(F_thrust/4.0f*Newton2g, 0.0f, f_max*0.9f); // Clamp thrust to prevent control saturation
        f_roll_g = M.x/(4.0f*dp)*Newton2g;
        f_pitch_g = M.y/(4.0f*dp)*Newton2g;
        f_yaw_g = M.z/(4.0f*c_tf)*Newton2g;

        // THESE CONNECT TO POWER_DISTRIBUTION_STOCK.C
        control->thrust = f_thrust_g;                   // This gets passed to firmware EKF
        control->roll = (int16_t)(f_roll_g*1e3f);
        control->pitch = (int16_t)(f_pitch_g*1e3f);
        control->yaw = (int16_t)(f_yaw_g*1e3f);

        // ADD RESPECTIVE THRUST COMPONENTS
        M1_thrust = clamp(f_thrust_g + f_roll_g - f_pitch_g + f_yaw_g, 0.0f, f_max);
        M2_thrust = clamp(f_thrust_g + f_roll_g + f_pitch_g - f_yaw_g, 0.0f, f_max);
        M3_thrust = clamp(f_thrust_g - f_roll_g + f_pitch_g + f_yaw_g, 0.0f, f_max);
        M4_thrust = clamp(f_thrust_g - f_roll_g - f_pitch_g - f_yaw_g, 0.0f, f_max);


        // TUMBLE DETECTION
        if(b3.z <= 0 && tumble_detection == true){ // If b3 axis has a negative z-component (Quadrotor is inverted)
            tumbled = true;
        }


        // UPDATE THRUST COMMANDS
        if(motorstop_flag || tumbled) // STOP MOTOR COMMANDS
        { 
            M1_thrust = 0.0f;
            M2_thrust = 0.0f;
            M3_thrust = 0.0f;
            M4_thrust = 0.0f;

        }
        else if(customThrust_flag) // REPLACE THRUST VALUES WITH CUSTOM VALUES
        {
            
            M1_thrust = thrust_override[0];
            M2_thrust = thrust_override[1];
            M3_thrust = thrust_override[2];
            M4_thrust = thrust_override[3];

        }

        // UPDATE PWM COMMANDS
        if(customPWM_flag)
        {
            M1_pwm = PWM_override[0]; 
            M2_pwm = PWM_override[1];
            M3_pwm = PWM_override[2];
            M4_pwm = PWM_override[3];
        }
        else 
        {
            // CONVERT THRUSTS TO PWM SIGNALS
            M1_pwm = (int32_t)thrust2PWM(M1_thrust); 
            M2_pwm = (int32_t)thrust2PWM(M2_thrust);
            M3_pwm = (int32_t)thrust2PWM(M3_thrust);
            M4_pwm = (int32_t)thrust2PWM(M4_thrust);
        }

    }
 

}