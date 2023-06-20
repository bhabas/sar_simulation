#include "Controller_GTC.h"


uint8_t value1 = 5;
void appMain() {

    while (1)
    {
        #ifdef CONFIG_SAR_SIM

            if (CTRL_Cmd.cmd_rx == true)
            {
                CTRL_Command(&CTRL_Cmd);
                CTRL_Cmd.cmd_rx = false;
            }


        #elif CONFIG_SAR_EXP

            if (appchannelReceiveDataPacket(&CTRL_Cmd,sizeof(CTRL_Cmd),APPCHANNEL_WAIT_FOREVER))
            {
                if (CTRL_Cmd.cmd_rx == true)
                {
                    CTRL_Command(&CTRL_Cmd);
                    CTRL_Cmd.cmd_rx = false;
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
    X_input = nml_mat_new(3,1);
    Y_output = nml_mat_new(4,1);

    // INIT DEEP RL NN POLICY
    NN_init(&NN_DeepRL,NN_Params_DeepRL);
    


    consolePrintf("GTC Controller Initiated\n");
}


void controllerOutOfTreeReset() {

    consolePrintf("GTC Controller Reset\n");
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
    Traj_Type = NONE;

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


    calcPlaneNormal(Plane_Angle);

}


void controllerOutOfTree(control_t *control,const setpoint_t *setpoint, 
                                            const sensorData_t *sensors, 
                                            const state_t *state, 
                                            const uint32_t tick) 
{

    // OPTICAL FLOW UPDATES
    if (RATE_DO_EXECUTE(100, tick)) {

        // UPDATE POS AND VEL
        r_BO = mkvec(state->position.x, state->position.y, state->position.z);
        V_BO = mkvec(state->velocity.x, state->velocity.y, state->velocity.z);

        // CALC DISPLACEMENT FROM PLANE CENTER
        r_PB = vsub(r_PO,r_BO); 

        // CALC RELATIVE DISTANCE AND VEL
        D_perp = vdot(r_PB,n_hat) + 1e-6f;

        V_perp = vdot(V_BO,n_hat);
        V_tx = vdot(V_BO,t_x);
        V_ty = vdot(V_BO,t_y);

        if (fabsf(D_perp) < 0.02f)
        {
            D_perp = 0.0f;
        }

        // CALC OPTICAL FLOW VALUES
        Theta_x = clamp(V_tx/D_perp,-20.0f,20.0f);
        Theta_y = clamp(V_ty/D_perp,-20.0f,20.0f);
        Theta_z = clamp(V_perp/D_perp,-20.0f,20.0f);
        Tau = clamp(1/Theta_z,0.0f,5.0f);

    }

    // TRAJECTORY UPDATES
    if (RATE_DO_EXECUTE(RATE_100_HZ, tick)) {

        switch (Traj_Type)
        {
            case NONE:
                /* DO NOTHING */
                break;

            case P2P:
                point2point_Traj();
                break;

            case CONST_VEL:
                const_velocity_Traj();
                break;

            case CONST_VEL_GZ:
                const_velocity_GZ_Traj();
                break;
        }
    }

    // POLICY UPDATES
    if (RATE_DO_EXECUTE(RATE_100_HZ, tick)) {

        X_input->data[0][0] = Tau;
        X_input->data[1][0] = Theta_x;
        X_input->data[2][0] = D_perp; 
        

        if(policy_armed_flag == true){
            
            switch (Policy)
            {
                case PARAM_OPTIM:

                    // EXECUTE POLICY IF TRIGGERED
                    if(Tau <= Policy_Trg_Action && onceFlag == false && V_perp > 0.1f){

                        onceFlag = true;

                        // UPDATE AND RECORD FLIP VALUES
                        flip_flag = true;  
                        statePos_tr = statePos;
                        stateVel_tr = stateVel;
                        stateQuat_tr = stateQuat;
                        stateOmega_tr = stateOmega;

                        Tau_tr = Tau;
                        Theta_x_tr = Theta_x_tr;
                        Theta_y_tr = Theta_y_tr;
                        D_perp_tr = D_perp;

                    
                        M_d.x = 0.0f;
                        M_d.y = -Policy_Flip_Action*1e-3f;
                        M_d.z = 0.0f;

                        F_thrust_flip = 0.0;
                        M_x_flip = M_d.x*1e3f;
                        M_y_flip = M_d.y*1e3f;
                        M_z_flip = M_d.z*1e3f;
                        }
                        
                    break;

                case DEEP_RL_ONBOARD:

                    // PASS OBSERVATION THROUGH POLICY NN
                    NN_forward(X_input,Y_output,&NN_DeepRL);

                    // SAMPLE POLICY TRIGGER ACTION
                    Policy_Trg_Action = GaussianSample(Y_output->data[0][0],exp(Y_output->data[2][0]));

                    // EXECUTE POLICY
                    if(Policy_Trg_Action >= Policy_Flip_threshold && onceFlag == false && V_perp > 0.1f){

                        onceFlag = true;

                        // SAMPLE AND SCALE BODY FLIP ACTION
                        Policy_Flip_Action = GaussianSample(Y_output->data[1][0],exp(Y_output->data[3][0]));
                        Policy_Flip_Action = scale_tanhAction(Policy_Flip_Action,ACTION_MIN,ACTION_MAX);

                        // UPDATE AND RECORD FLIP VALUES
                        flip_flag = true;  
                        statePos_tr = statePos;
                        stateVel_tr = stateVel;
                        stateQuat_tr = stateQuat;
                        stateOmega_tr = stateOmega;

                        Tau_tr = Tau;
                        Theta_x_tr = Theta_x_tr;
                        Theta_y_tr = Theta_y_tr;
                        D_perp_tr = D_perp;

                    
                        M_d.x = 0.0f;
                        M_d.y = -Policy_Flip_Action*1e-3f;
                        M_d.z = 0.0f;

                        F_thrust_flip = 0.0;
                        M_x_flip = M_d.x*1e3f;
                        M_y_flip = M_d.y*1e3f;
                        M_z_flip = M_d.z*1e3f;
                        }
                        
                    break;

                case DEEP_RL_SB3:

                    break;
                    
            default:
                break;
            }

        }

    }
    
    // CTRL UPDATES
    if (RATE_DO_EXECUTE(RATE_100_HZ, tick)) {


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

PARAM_GROUP_START(CTRL_Params)
// NOTE: PARAM GROUP + NAME + 1 CANNOT EXCEED 26 CHARACTERS (WHY? IDK.)
// NOTE: CANNOT HAVE A LOG AND A PARAM ACCESS THE SAME VALUE

PARAM_ADD(PARAM_FLOAT, CF_mass, &m)
PARAM_ADD(PARAM_FLOAT, Ixx, &Ixx)
PARAM_ADD(PARAM_FLOAT, Iyy, &Iyy)
PARAM_ADD(PARAM_FLOAT, Izz, &Izz)

PARAM_ADD(PARAM_FLOAT, Prop_Dist, &Prop_Dist)
PARAM_ADD(PARAM_FLOAT, C_tf, &C_tf)
PARAM_ADD(PARAM_FLOAT, f_max, &f_max)

PARAM_ADD(PARAM_UINT8, SafeMode, &safeModeEnable)
PARAM_ADD(PARAM_UINT8, PolicyType, &Policy)

PARAM_ADD(PARAM_FLOAT, P_kp_xy, &P_kp_xy)
PARAM_ADD(PARAM_FLOAT, P_kd_xy, &P_kd_xy) 
PARAM_ADD(PARAM_FLOAT, P_ki_xy, &P_ki_xy)
PARAM_ADD(PARAM_FLOAT, i_range_xy, &i_range_xy)

PARAM_ADD(PARAM_FLOAT, P_kp_z,  &P_kp_z)
PARAM_ADD(PARAM_FLOAT, P_kd_z,  &P_kd_z)
PARAM_ADD(PARAM_FLOAT, P_ki_z,  &P_ki_z)
PARAM_ADD(PARAM_FLOAT, i_range_z, &i_range_z)

PARAM_ADD(PARAM_FLOAT, R_kp_xy, &R_kp_xy)
PARAM_ADD(PARAM_FLOAT, R_kd_xy, &R_kd_xy) 
PARAM_ADD(PARAM_FLOAT, R_ki_xy, &R_ki_xy)
PARAM_ADD(PARAM_FLOAT, i_range_R_xy, &i_range_R_xy)

PARAM_ADD(PARAM_FLOAT, R_kp_z,  &R_kp_z)
PARAM_ADD(PARAM_FLOAT, R_kd_z,  &R_kd_z)
PARAM_ADD(PARAM_FLOAT, R_ki_z,  &R_ki_z)
PARAM_ADD(PARAM_FLOAT, i_range_R_z, &i_range_R_z)
PARAM_GROUP_STOP(CTRL_Params)

LOG_GROUP_START(States_CTRL)
LOG_ADD(LOG_UINT32, Z_xy,   &StatesZ_CTRL.xy)
LOG_ADD(LOG_INT16,  Z_z,    &StatesZ_CTRL.z)

LOG_ADD(LOG_UINT32, Z_vxy,  &StatesZ_CTRL.vxy)
LOG_ADD(LOG_INT16,  Z_vz,   &StatesZ_CTRL.vz)

LOG_ADD(LOG_UINT32, Z_quat, &StatesZ_CTRL.quat)

LOG_ADD(LOG_UINT32, Z_wxy,  &StatesZ_CTRL.wxy)
LOG_ADD(LOG_INT16,  Z_wz,   &StatesZ_CTRL.wz)

LOG_ADD(LOG_UINT32, Z_Thetaxy, &StatesZ_CTRL.Theta_xy)
LOG_ADD(LOG_INT16,  Z_Tau,  &StatesZ_CTRL.Tau)
LOG_ADD(LOG_INT16,  Z_D_perp, &StatesZ_CTRL.D_perp)

LOG_ADD(LOG_UINT32, Z_FMz, &StatesZ_CTRL.FMz)
LOG_ADD(LOG_UINT32, Z_Mxy, &StatesZ_CTRL.Mxy)

LOG_ADD(LOG_UINT32, Z_f_12, &StatesZ_CTRL.M_thrust12)
LOG_ADD(LOG_UINT32, Z_f_34, &StatesZ_CTRL.M_thrust34)

LOG_ADD(LOG_UINT32, Z_PWM12, &StatesZ_CTRL.MS_PWM12)
LOG_ADD(LOG_UINT32, Z_PWM34, &StatesZ_CTRL.MS_PWM34)

LOG_ADD(LOG_UINT32, Z_NN_FP, &StatesZ_CTRL.NN_FP)

LOG_ADD(LOG_UINT8, Z_vbat, &value1)
LOG_GROUP_STOP(States_CTRL)



LOG_GROUP_START(SetPoints_CTRL)
LOG_ADD(LOG_UINT32, Z_xy,   &setpointZ_CTRL.xy)
LOG_ADD(LOG_INT16,  Z_z,    &setpointZ_CTRL.z)

LOG_ADD(LOG_UINT32, Z_vxy,  &setpointZ_CTRL.vxy)
LOG_ADD(LOG_INT16,  Z_vz,   &setpointZ_CTRL.vz)

LOG_ADD(LOG_UINT32, Z_axy,  &setpointZ_CTRL.axy)
LOG_ADD(LOG_INT16,  Z_az,   &setpointZ_CTRL.az)
LOG_GROUP_STOP(SetPoints_CTRL)


LOG_GROUP_START(FlipData_CTRL)
LOG_ADD(LOG_UINT32, Z_xy,   &FlipStatesZ_CTRL.xy)
LOG_ADD(LOG_INT16,  Z_z,    &FlipStatesZ_CTRL.z)

LOG_ADD(LOG_UINT32, Zf_vxy,  &FlipStatesZ_CTRL.vxy)
LOG_ADD(LOG_INT16,  Z_vz,   &FlipStatesZ_CTRL.vz)

LOG_ADD(LOG_UINT32, Z_quat, &FlipStatesZ_CTRL.quat)

LOG_ADD(LOG_UINT32, Z_wxy,  &FlipStatesZ_CTRL.wxy)
LOG_ADD(LOG_INT16,  Z_wz,   &FlipStatesZ_CTRL.wz)

LOG_ADD(LOG_UINT32, Z_Thetaxy, &FlipStatesZ_CTRL.Theta_xy)
LOG_ADD(LOG_INT16,  Z_Tau,  &FlipStatesZ_CTRL.Tau)
LOG_ADD(LOG_INT16,  Z_D_perp, &FlipStatesZ_CTRL.D_perp)

LOG_ADD(LOG_UINT8, Flip_Flag, &flip_flag)
LOG_GROUP_STOP(FlipData_CTRL)


LOG_GROUP_START(CTRL_Flags)
LOG_ADD(LOG_FLOAT, Pos_Ctrl, &kp_xf)
LOG_ADD(LOG_FLOAT, Vel_Ctrl, &kd_xf)
LOG_ADD(LOG_UINT8, Motorstop, &motorstop_flag)
LOG_ADD(LOG_UINT8, Tumbled, &tumbled)
LOG_ADD(LOG_UINT8, Tumble_Detect, &tumble_detection)
LOG_ADD(LOG_UINT8, Moment, &moment_flag)
LOG_ADD(LOG_UINT8, Pol_Armed, &policy_armed_flag)
LOG_ADD(LOG_FLOAT, Pol_Trg_Act, &Policy_Trg_Action)
LOG_ADD(LOG_FLOAT, Pol_Flip_Act, &Policy_Flip_Action)
LOG_GROUP_STOP(CTRL_Flags)
#endif