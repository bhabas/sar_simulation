#include "controller_GTC.h"


uint8_t value1 = 5;
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

    
bool controllerOutOfTreeTest() {

  return true;
}


void controllerOutOfTreeInit() {

    #ifdef CONFIG_SAR_EXP
    ledSet(LED_BLUE_L, 0);
    ledSet(LED_BLUE_NRF, 0);
    #endif

    controllerOutOfTreeReset();
    controllerOutOfTreeTest();
    J = mdiag(Ixx,Iyy,Izz);

    consolePrintf("GTC Initiated\n");

}


void controllerOutOfTreeReset() {

    consolePrintf("GTC Reset\n");
    consolePrintf("Policy_Type: %d\n",Policy);

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
    policy_armed_flag = false;
    flip_flag = false;
    onceFlag = false;


    // RESET LOGGED FLIP VALUES
    statePos_tr = vzero();
    stateVel_tr = vzero();
    stateQuat_tr = mkquat(0.0f,0.0f,0.0f,1.0f);
    stateOmega_tr = vzero();

    Tau_tr = 0.0f;
    Theta_x_tr = 0.0f;
    Theta_y_tr = 0.0f;
    D_perp_tr = 0.0f;

    Policy_Flip_tr = 0.0f;
    Policy_Action_tr = 0.0f;

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
        f_roll_g = M.x/(4.0f*Prop_Dist)*Newton2g;
        f_pitch_g = M.y/(4.0f*Prop_Dist)*Newton2g;
        f_yaw_g = M.z/(4.0f*C_tf)*Newton2g;

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

        compressStates();
        compressSetpoints();
        compressFlipStates();

        #ifdef CONFIG_SAR_EXP
        if(safeModeEnable)
        {
            motorsSetRatio(MOTOR_M1, 0);
            motorsSetRatio(MOTOR_M2, 0);
            motorsSetRatio(MOTOR_M3, 0);
            motorsSetRatio(MOTOR_M4, 0);
        }
        else{
            // SEND PWM VALUES TO MOTORS
            motorsSetRatio(MOTOR_M1, M4_pwm);
            motorsSetRatio(MOTOR_M2, M3_pwm);
            motorsSetRatio(MOTOR_M3, M2_pwm);
            motorsSetRatio(MOTOR_M4, M1_pwm);
        }
        #endif


    }
 

}


#ifdef CONFIG_SAR_EXP
LOG_GROUP_START(LogStates_GTC)
LOG_ADD(LOG_UINT32, Z_xy,   &StatesZ_GTC.xy)
LOG_ADD(LOG_INT16,  Z_z,    &StatesZ_GTC.z)

LOG_ADD(LOG_UINT32, Z_vxy,  &StatesZ_GTC.vxy)
LOG_ADD(LOG_INT16,  Z_vz,   &StatesZ_GTC.vz)

LOG_ADD(LOG_UINT32, Z_quat, &StatesZ_GTC.quat)

LOG_ADD(LOG_UINT32, Z_wxy,  &StatesZ_GTC.wxy)
LOG_ADD(LOG_INT16,  Z_wz,   &StatesZ_GTC.wz)

LOG_ADD(LOG_UINT32, Z_Thetaxy, &StatesZ_GTC.Theta_xy)
LOG_ADD(LOG_INT16,  Z_Tau,  &StatesZ_GTC.Tau)
LOG_ADD(LOG_INT16,  Z_D_perp, &StatesZ_GTC.D_perp)

LOG_ADD(LOG_UINT32, Z_FMz, &StatesZ_GTC.FMz)
LOG_ADD(LOG_UINT32, Z_Mxy, &StatesZ_GTC.Mxy)

LOG_ADD(LOG_UINT32, Z_f_12, &StatesZ_GTC.M_thrust12)
LOG_ADD(LOG_UINT32, Z_f_34, &StatesZ_GTC.M_thrust34)

LOG_ADD(LOG_UINT32, Z_PWM12, &StatesZ_GTC.MS_PWM12)
LOG_ADD(LOG_UINT32, Z_PWM34, &StatesZ_GTC.MS_PWM34)

LOG_ADD(LOG_UINT32, Z_NN_FP, &StatesZ_GTC.NN_FP)

LOG_ADD(LOG_UINT8, Z_vbat, &value1)
LOG_GROUP_STOP(LogStateData_GTC)



LOG_GROUP_START(LogSetPoints_GTC)
LOG_ADD(LOG_UINT32, Z_xy,   &setpointZ_GTC.xy)
LOG_ADD(LOG_INT16,  Z_z,    &setpointZ_GTC.z)

LOG_ADD(LOG_UINT32, Z_vxy,  &setpointZ_GTC.vxy)
LOG_ADD(LOG_INT16,  Z_vz,   &setpointZ_GTC.vz)

LOG_ADD(LOG_UINT32, Z_axy,  &setpointZ_GTC.axy)
LOG_ADD(LOG_INT16,  Z_az,   &setpointZ_GTC.az)
LOG_GROUP_STOP(LogSetPoints_GTC)


LOG_GROUP_START(LogFlipData_GTC)
LOG_ADD(LOG_UINT32, Z_xy,   &FlipStatesZ_GTC.xy)
LOG_ADD(LOG_INT16,  Z_z,    &FlipStatesZ_GTC.z)

LOG_ADD(LOG_UINT32, Zf_vxy,  &FlipStatesZ_GTC.vxy)
LOG_ADD(LOG_INT16,  Z_vz,   &FlipStatesZ_GTC.vz)

LOG_ADD(LOG_UINT32, Z_quat, &FlipStatesZ_GTC.quat)

LOG_ADD(LOG_UINT32, Z_wxy,  &FlipStatesZ_GTC.wxy)
LOG_ADD(LOG_INT16,  Z_wz,   &FlipStatesZ_GTC.wz)

LOG_ADD(LOG_UINT32, Z_Thetaxy, &FlipStatesZ_GTC.Theta_xy)
LOG_ADD(LOG_INT16,  Z_Tau,  &FlipStatesZ_GTC.Tau)
LOG_ADD(LOG_INT16,  Z_D_perp, &FlipStatesZ_GTC.D_perp)

LOG_ADD(LOG_UINT8, Flip_Flag, &flip_flag)
LOG_GROUP_STOP(LogFlipData_GTC)


LOG_GROUP_START(valsLog)
LOG_ADD(LOG_UINT8, Motorstop_Flag, &motorstop_flag)
LOG_ADD(LOG_FLOAT, Pos_Ctrl_Flag, &kp_xf)
LOG_ADD(LOG_FLOAT, Vel_Ctrl_Flag, &kd_xf)
// LOG_ADD(LOG_UINT8, Execute_Traj_Flag, &execute_vel_traj)
LOG_ADD(LOG_UINT8, Tumbled_Flag, &tumbled)
LOG_ADD(LOG_UINT8, Tumble_Detect, &tumble_detection)
LOG_ADD(LOG_UINT8, Moment_Flag, &moment_flag)
LOG_ADD(LOG_UINT8, Policy_Armed_Flag, &policy_armed_flag)
LOG_GROUP_STOP(valsLog)
#endif