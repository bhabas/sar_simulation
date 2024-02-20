#include "Controller_GTC.h"

#define max(a,b) ((a) > (b) ? (a) : (b))


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
    
    #endif

    controllerOutOfTreeReset();
    controllerOutOfTreeTest();

    // INIT DEEP RL NN POLICY
    X_input = nml_mat_new(4,1);
    Y_output = nml_mat_new(4,1);

    // INIT DEEP RL NN POLICY
    // DEBUG_PRINT("Free heap: %d bytes\n", xPortGetFreeHeapSize());
    NN_init(&NN_DeepRL,NN_Params_DeepRL);
    // DEBUG_PRINT("Free heap: %d bytes\n", xPortGetFreeHeapSize());

    

    consolePrintf("GTC Controller Initiated\n");
}


void controllerOutOfTreeReset() {

    consolePrintf("GTC Controller Reset\n");
    consolePrintf("SAR_Type: %d\n",SAR_Type);
    consolePrintf("Policy_Type: %d\n",Policy);
    J = mdiag(Ixx,Iyy,Izz);

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
    Tumbled_Flag = false;
    CustomThrust_Flag = false;
    CustomMotorCMD_Flag = false;
    AngAccel_Flag = false;

    // RESET TRAJECTORY FLAGS
    Traj_Type = NONE;

    // RESET POLICY FLAGS
    Policy_Armed_Flag = false;
    Trg_Flag = false;
    onceFlag = false;

    // UPDATE COLLISION RADIUS
    Collision_Radius = max(L_eff,Forward_Reach);


    // RESET LOGGED TRIGGER VALUES
    Trg_Flag = false;
    Pos_B_O_trg = vzero();
    Vel_B_O_trg = vzero();
    Quat_B_O_trg = mkquat(0.0f,0.0f,0.0f,1.0f);
    Omega_B_O_trg = vzero();

    Pos_P_B_trg = vzero();
    Vel_B_P_trg = vzero();
    Quat_P_B_trg = mkquat(0.0f,0.0f,0.0f,1.0f);
    Omega_B_P_trg = vzero();

    Theta_x_trg = 0.0f;
    Theta_y_trg = 0.0f;
    Tau_trg = 0.0f;
    Tau_CR_trg = 0.0f;

    Policy_Trg_Action_trg = 0.0f;
    Policy_Rot_Action_trg = 0.0f;


}


void controllerOutOfTree(control_t *control,const setpoint_t *setpoint, 
                                            const sensorData_t *sensors, 
                                            const state_t *state, 
                                            const uint32_t tick) 
{

    // CHECK FOR CRAZYSWARM SIGNAL
    #ifdef CONFIG_SAR_EXP
    if (RATE_DO_EXECUTE(RATE_25_HZ, tick))
    {
        uint32_t now = xTaskGetTickCount();
        if (now - PrevCrazyswarmTick > 3000)
        {
            Armed_Flag = false;
        }
    }
    #endif

    // POLICY UPDATES
    if (isOFUpdated == true) {

        isOFUpdated = false;

        if(Policy_Armed_Flag == true){


            switch (Policy)
            {
                case PARAM_OPTIM:

                    // EXECUTE POLICY IF TRIGGERED
                    if(Tau_CR <= Policy_Trg_Action && onceFlag == false && abs(Tau_CR) <= 3.0f){

                        onceFlag = true;

                        // UPDATE AND RECORD TRIGGER VALUES
                        Trg_Flag = true;  
                        Pos_B_O_trg = Pos_B_O;
                        Vel_B_O_trg = Vel_B_O;
                        Quat_B_O_trg = Quat_B_O;
                        Omega_B_O_trg = Omega_B_O;

                        Pos_P_B_trg = Pos_P_B;
                        Vel_B_P_trg = Vel_B_P;
                        Quat_P_B_trg = Quat_P_B;
                        Omega_B_P_trg = Omega_B_P;

                        D_perp_trg = D_perp;
                        Vel_mag_B_P_trg = Vel_mag_B_P;
                        Vel_angle_B_P_trg = Vel_angle_B_P;

                        Tau_trg = Tau;
                        Tau_CR_trg = Tau_CR;
                        Theta_x_trg = Theta_x;
                        Theta_y_trg = Theta_y;

                        Policy_Trg_Action_trg = Policy_Trg_Action;
                        Policy_Rot_Action_trg = Policy_Rot_Action;

                        M_d.x = 0.0f;
                        M_d.y = Policy_Rot_Action*Iyy;
                        M_d.z = 0.0f;
                        }
                        
                    break;

                case DEEP_RL_SB3:

                    // EXECUTE POLICY IF TRIGGERED
                    if(onceFlag == false){

                        onceFlag = true;

                        // UPDATE AND RECORD TRIGGER VALUES
                        Trg_Flag = true;  
                        Pos_B_O_trg = Pos_B_O;
                        Vel_B_O_trg = Vel_B_O;
                        Quat_B_O_trg = Quat_B_O;
                        Omega_B_O_trg = Omega_B_O;

                        Pos_P_B_trg = Pos_P_B;
                        Vel_B_P_trg = Vel_B_P;
                        Quat_P_B_trg = Quat_P_B;
                        Omega_B_P_trg = Omega_B_P;

                        Tau_trg = Tau;
                        Tau_CR_trg = Tau_CR;
                        Theta_x_trg = Theta_x;
                        Theta_y_trg = Theta_y;
                        D_perp_trg = D_perp;
                        D_perp_CR_trg = D_perp_CR;


                        Policy_Trg_Action_trg = Policy_Trg_Action;
                        Policy_Rot_Action_trg = Policy_Rot_Action;

                        M_d.x = 0.0f;
                        M_d.y = Policy_Rot_Action*Iyy;
                        M_d.z = 0.0f;
                    }

                    break;

                case DEEP_RL_ONBOARD:

                    // // PASS OBSERVATION THROUGH POLICY NN
                    // NN_forward(X_input,Y_output,&NN_DeepRL);

                    // // SAMPLE POLICY TRIGGER ACTION
                    // Policy_Trg_Action = GaussianSample(Y_output->data[0][0],exp(Y_output->data[2][0]));

             
                    //     }
                        
                    break;

                
                    
            default:
                break;
            }

        }

    }


    if (RATE_DO_EXECUTE(RATE_25_HZ, tick))
    {
        updateRotationMatrices();
    }

    // if (RATE_DO_EXECUTE(100, tick))
    // {
    //     NN_forward(X_input,Y_output,&NN_DeepRL);
    //     NN_forward(X_input,Y_output,&NN_DeepRL);
    //     NN_forward(X_input,Y_output,&NN_DeepRL);

    //     Policy_Trg_Action = Y_output->data[0][0]+0.00001f*tick;
    //     // nml_mat_print_CF(Y_output);
    // }

    


    
    // STATE UPDATES
    if (RATE_DO_EXECUTE(RATE_100_HZ, tick)) {

        float time_delta = (tick-prev_tick)/1000.0f;

        // CALC STATES WRT ORIGIN
        Pos_B_O = mkvec(state->position.x, state->position.y, state->position.z);          // [m]
        Vel_B_O = mkvec(state->velocity.x, state->velocity.y, state->velocity.z);          // [m/s]
        Accel_B_O = mkvec(sensors->acc.x*9.81f, sensors->acc.y*9.81f, sensors->acc.z*9.81f); // [m/s^2]
        Accel_B_O_Mag = firstOrderFilter(vmag(Accel_B_O),Accel_B_O_Mag,0.5f);

        

        Omega_B_O = mkvec(radians(sensors->gyro.x), radians(sensors->gyro.y), radians(sensors->gyro.z));   // [rad/s]

        // CALC AND FILTER ANGULAR ACCELERATION
        dOmega_B_O.x = firstOrderFilter((Omega_B_O.x - Omega_B_O_prev.x)/time_delta,dOmega_B_O.x,0.90f); // [rad/s^2]
        dOmega_B_O.y = firstOrderFilter((Omega_B_O.y - Omega_B_O_prev.y)/time_delta,dOmega_B_O.y,0.90f); // [rad/s^2]
        dOmega_B_O.z = firstOrderFilter((Omega_B_O.z - Omega_B_O_prev.z)/time_delta,dOmega_B_O.z,0.90f); // [rad/s^2]


        Quat_B_O = mkquat(state->attitudeQuaternion.x,
                        state->attitudeQuaternion.y,
                        state->attitudeQuaternion.z,
                        state->attitudeQuaternion.w);

        // CALC STATES WRT PLANE
        Pos_P_B = mvmul(R_WP,vsub(r_P_O,Pos_B_O)); 
        Vel_B_P = mvmul(R_WP,Vel_B_O);
        Vel_mag_B_P = vmag(Vel_B_P);
        Vel_angle_B_P = atan2f(Vel_B_P.z,Vel_B_P.x)*Rad2Deg;
        Omega_B_P = Omega_B_O;



        // if (Accel_B_O_Mag > 10.0f && Impact_Flag_OB == false)
        // {
        //     Impact_Flag_OB = true;
        //     Pos_B_O_impact_OB = Pos_B_O;
        //     Vel_B_P_impact_OB = Vel_B_P;
        //     Quat_B_O_impact_OB = Quat_B_O;
        //     Omega_B_P_impact_OB = Omega_B_P;
        //     Accel_B_O_Mag_impact_OB = Accel_B_O_Mag;
        // }


        // SAVE PREVIOUS VALUES
        Omega_B_O_prev = Omega_B_O;
        prev_tick = tick;
    }

    // OPTICAL FLOW UPDATES
    if (RATE_DO_EXECUTE(RATE_100_HZ, tick))
    {
        // UPDATE GROUND TRUTH OPTICAL FLOW
        updateOpticalFlowAnalytic(state,sensors);

        // POLICY VECTOR UPDATE
        if (CamActive_Flag == true)
        {
            // ONLY UPDATE WITH NEW OPTICAL FLOW DATA
            isOFUpdated = updateOpticalFlowEst();

            // UPDATE POLICY VECTOR
            X_input->data[0][0] = Tau_Cam;
            X_input->data[1][0] = Theta_x_Cam;
            X_input->data[2][0] = D_perp; 
            X_input->data[3][0] = Plane_Angle_deg; 
        }
        else
        {
            // UPDATE AT THE ABOVE FREQUENCY
            isOFUpdated = true;

            // // UPDATE POLICY VECTOR
            // X_input->data[0][0] = Tau;
            // X_input->data[1][0] = Theta_x;
            // X_input->data[2][0] = D_perp; 
            // X_input->data[3][0] = Plane_Angle_deg;
        }
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
        
    // CTRL UPDATES
    if (RATE_DO_EXECUTE(RATE_100_HZ, tick)) {


        controlOutput(state,sensors);

        if(AngAccel_Flag == true || Trg_Flag == true)
        {
            F_thrust = 0.0f;
            M = vscl(2.0f,M_d);
        }

        
        // MOTOR MIXING (GTC_Derivation_V2.pdf) 
        M1_thrust = F_thrust * Prop_23_x/(Prop_14_x + Prop_23_x) - M.x * 1/(Prop_14_y + Prop_23_y) - M.y * 1/(Prop_14_x + Prop_23_x) - M.z * Prop_23_y/(C_tf*(Prop_14_y + Prop_23_y));
        M2_thrust = F_thrust * Prop_14_x/(Prop_14_x + Prop_23_x) - M.x * 1/(Prop_14_y + Prop_23_y) + M.y * 1/(Prop_14_x + Prop_23_x) + M.z * Prop_14_y/(C_tf*(Prop_14_y + Prop_23_y));
        M3_thrust = F_thrust * Prop_14_x/(Prop_14_x + Prop_23_x) + M.x * 1/(Prop_14_y + Prop_23_y) + M.y * 1/(Prop_14_x + Prop_23_x) - M.z * Prop_14_y/(C_tf*(Prop_14_y + Prop_23_y));
        M4_thrust = F_thrust * Prop_23_x/(Prop_14_x + Prop_23_x) + M.x * 1/(Prop_14_y + Prop_23_y) - M.y * 1/(Prop_14_x + Prop_23_x) + M.z * Prop_23_y/(C_tf*(Prop_14_y + Prop_23_y));


        // CLAMP AND CONVERT THRUST FROM [N] AND [N*M] TO [g]
        M1_thrust = clamp((M1_thrust/2.0f)*Newton2g,0.0f,Thrust_max);
        M2_thrust = clamp((M2_thrust/2.0f)*Newton2g,0.0f,Thrust_max);
        M3_thrust = clamp((M3_thrust/2.0f)*Newton2g,0.0f,Thrust_max);
        M4_thrust = clamp((M4_thrust/2.0f)*Newton2g,0.0f,Thrust_max);

        // TUMBLE DETECTION
        if(b3.z <= 0 && TumbleDetect_Flag == true){ // If b3 axis has a negative z-component (Quadrotor is inverted)
            Tumbled_Flag = true;
        }

        
        if(CustomThrust_Flag) // REPLACE THRUST VALUES WITH CUSTOM VALUES
        {
            M1_thrust = thrust_override[0];
            M2_thrust = thrust_override[1];
            M3_thrust = thrust_override[2];
            M4_thrust = thrust_override[3];

            // CONVERT THRUSTS TO M_CMD SIGNALS
            M1_CMD = (int32_t)thrust2Motor_CMD(M1_thrust); 
            M2_CMD = (int32_t)thrust2Motor_CMD(M2_thrust);
            M3_CMD = (int32_t)thrust2Motor_CMD(M3_thrust);
            M4_CMD = (int32_t)thrust2Motor_CMD(M4_thrust);
        }
        else if(CustomMotorCMD_Flag)
        {
            M1_CMD = M_CMD_override[0]; 
            M2_CMD = M_CMD_override[1];
            M3_CMD = M_CMD_override[2];
            M4_CMD = M_CMD_override[3];
        }
        else 
        {
            // CONVERT THRUSTS TO M_CMD SIGNALS
            M1_CMD = (int32_t)thrust2Motor_CMD(M1_thrust); 
            M2_CMD = (int32_t)thrust2Motor_CMD(M2_thrust);
            M3_CMD = (int32_t)thrust2Motor_CMD(M3_thrust);
            M4_CMD = (int32_t)thrust2Motor_CMD(M4_thrust);
        }

        if (!Armed_Flag || MotorStop_Flag || Tumbled_Flag)
        {
            M1_thrust = 0.0f;
            M2_thrust = 0.0f;
            M3_thrust = 0.0f;
            M4_thrust = 0.0f;

            M1_CMD = 0.0f; 
            M2_CMD = 0.0f;
            M3_CMD = 0.0f;
            M4_CMD = 0.0f;
        }


        

        #ifdef CONFIG_SAR_EXP
        if(Armed_Flag)
        {
            // SEND CMD VALUES TO MOTORS
            motorsSetRatio(MOTOR_M1, M1_CMD);
            motorsSetRatio(MOTOR_M2, M2_CMD);
            motorsSetRatio(MOTOR_M3, M3_CMD);
            motorsSetRatio(MOTOR_M4, M4_CMD);
            
            // TURN ON ARMING LEDS
            ledSet(LED_BLUE_L, 1);
            ledSet(LED_BLUE_NRF, 1);
        }
        else{
            motorsSetRatio(MOTOR_M1, 0);
            motorsSetRatio(MOTOR_M2, 0);
            motorsSetRatio(MOTOR_M3, 0);
            motorsSetRatio(MOTOR_M4, 0);
            
            // TURN OFF ARMING LEDS
            ledSet(LED_BLUE_L, 0);
            ledSet(LED_BLUE_NRF, 0);
        }
        #endif

        // COMPRESS STATES
        compressStates();
        compressSetpoints();
        compressTrgStates();


    }

    
 

}


#ifdef CONFIG_SAR_EXP


// NOTE: PARAM GROUP + NAME + 1 CANNOT EXCEED 26 CHARACTERS (WHY? IDK.)
// NOTE: CANNOT HAVE A LOG AND A PARAM ACCESS THE SAME VALUE
PARAM_GROUP_START(System_Params)
PARAM_ADD(PARAM_FLOAT, Mass, &m)
PARAM_ADD(PARAM_FLOAT, Ixx, &Ixx)
PARAM_ADD(PARAM_FLOAT, Iyy, &Iyy)
PARAM_ADD(PARAM_FLOAT, Izz, &Izz)

PARAM_ADD(PARAM_FLOAT, Prop_14_x, &Prop_14_x)
PARAM_ADD(PARAM_FLOAT, Prop_14_y, &Prop_14_y)
PARAM_ADD(PARAM_FLOAT, Prop_23_x, &Prop_23_x)
PARAM_ADD(PARAM_FLOAT, Prop_23_y, &Prop_23_y)

PARAM_ADD(PARAM_FLOAT, C_tf, &C_tf)
PARAM_ADD(PARAM_FLOAT, Thrust_max, &Thrust_max)
PARAM_ADD(PARAM_FLOAT, L_eff, &L_eff)
PARAM_ADD(PARAM_FLOAT, Fwd_Reach, &Forward_Reach)


PARAM_ADD(PARAM_UINT8, Armed, &Armed_Flag)
PARAM_ADD(PARAM_UINT8, PolicyType, &Policy)
PARAM_ADD(PARAM_UINT8, SAR_Type, &SAR_Type)
PARAM_GROUP_STOP(System_Params)


PARAM_GROUP_START(CTRL_Params)
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



LOG_GROUP_START(Z_States)

LOG_ADD(LOG_UINT32, r_BOxy,         &States_Z.r_BOxy)
LOG_ADD(LOG_INT16,  r_BOz,          &States_Z.r_BOz)
LOG_ADD(LOG_UINT32, V_BOxy,         &States_Z.V_BOxy)
LOG_ADD(LOG_INT16,  V_BOz,          &States_Z.V_BOz)
LOG_ADD(LOG_UINT32, Acc_BOxy,       &States_Z.Acc_BOxy)
LOG_ADD(LOG_INT16,  Acc_BOz,        &States_Z.Acc_BOz)
LOG_ADD(LOG_UINT32, Quat_BO,        &States_Z.Quat_BO)
LOG_ADD(LOG_UINT32, Omega_BOxy,     &States_Z.Omega_BOxy)
LOG_ADD(LOG_INT16,  Omega_BOz,      &States_Z.Omega_BOz)
LOG_ADD(LOG_INT16,  dOmega_BOy,     &States_Z.dOmega_BOy)

LOG_ADD(LOG_UINT32, VelRel_BP,      &States_Z.VelRel_BP)
LOG_ADD(LOG_UINT32, r_PBxy,         &States_Z.r_PBxy)
LOG_ADD(LOG_INT16,  r_PBz,          &States_Z.r_PBz)

LOG_ADD(LOG_UINT32, D_perp,         &States_Z.D_perp)
LOG_ADD(LOG_UINT32, Tau,            &States_Z.Tau)
LOG_ADD(LOG_INT16,  Theta_x,        &States_Z.Theta_x)
LOG_ADD(LOG_UINT32, Pol_Actions,    &States_Z.Policy_Actions)

LOG_ADD(LOG_UINT32, FMz,            &States_Z.FMz)
LOG_ADD(LOG_UINT32, Mxy,            &States_Z.Mxy)
LOG_ADD(LOG_UINT32, f_12,           &States_Z.M_thrust12)
LOG_ADD(LOG_UINT32, f_34,           &States_Z.M_thrust34)
LOG_ADD(LOG_UINT32, M_CMD12,        &States_Z.M_CMD12)
LOG_ADD(LOG_UINT32, M_CMD34,        &States_Z.M_CMD34)
LOG_GROUP_STOP(Z_States)


LOG_GROUP_START(Z_SetPoints)
LOG_ADD(LOG_UINT32, x_xy,         &SetPoints_Z.xy)
LOG_ADD(LOG_INT16,  x_z,          &SetPoints_Z.z)

LOG_ADD(LOG_UINT32, v_xy,        &SetPoints_Z.vxy)
LOG_ADD(LOG_INT16,  v_z,         &SetPoints_Z.vz)

LOG_ADD(LOG_UINT32, a_xy,        &SetPoints_Z.axy)
LOG_ADD(LOG_INT16,  a_z,         &SetPoints_Z.az)
LOG_GROUP_STOP(Z_SetPoints)



LOG_GROUP_START(Z_Trg)
LOG_ADD(LOG_UINT8, Trg_Flag, &Trg_Flag)
LOG_ADD(LOG_UINT32, r_BOxy,         &TrgStates_Z.r_BOxy)
LOG_ADD(LOG_INT16,  r_BOz,          &TrgStates_Z.r_BOz)
LOG_ADD(LOG_UINT32, V_BOxy,         &TrgStates_Z.V_BOxy)
LOG_ADD(LOG_INT16,  V_BOz,          &TrgStates_Z.V_BOz)
LOG_ADD(LOG_UINT32, Quat_BO,        &TrgStates_Z.Quat_BO)
LOG_ADD(LOG_INT16, Omega_BOy,       &TrgStates_Z.Omega_BOy)
LOG_ADD(LOG_UINT32, VelRel_BP,      &TrgStates_Z.VelRel_BP)
LOG_ADD(LOG_UINT32, r_PBxy,         &TrgStates_Z.r_PBxy)
LOG_ADD(LOG_INT16,  r_PBz,          &TrgStates_Z.r_PBz)
LOG_ADD(LOG_UINT32, D_perp,         &TrgStates_Z.D_perp)
LOG_ADD(LOG_UINT32, Tau,            &TrgStates_Z.Tau)
LOG_ADD(LOG_INT16,  Theta_x,        &TrgStates_Z.Theta_x)
LOG_ADD(LOG_UINT32, Pol_Actions,    &TrgStates_Z.Policy_Actions)
LOG_GROUP_STOP(Z_Trg)


LOG_GROUP_START(Z_Impact)
// TBD IF NEEDED/CAPBABLE
LOG_GROUP_STOP(Z_Impact)



LOG_GROUP_START(Misc)
LOG_ADD(LOG_FLOAT, Pos_Ctrl,        &kp_xf)
LOG_ADD(LOG_FLOAT, Vel_Ctrl,        &kd_xf)
LOG_ADD(LOG_UINT8, Motorstop,       &MotorStop_Flag)
LOG_ADD(LOG_UINT8, Tumbled_Flag,    &Tumbled_Flag)
LOG_ADD(LOG_UINT8, Tumble_Detect,   &TumbleDetect_Flag)
LOG_ADD(LOG_UINT8, AngAccel_Flag,   &AngAccel_Flag)
LOG_ADD(LOG_UINT8, Armed_Flag,      &Armed_Flag)
LOG_ADD(LOG_UINT8, Policy_Armed,    &Policy_Armed_Flag)
LOG_ADD(LOG_UINT8, CustomThrust,    &CustomThrust_Flag)
LOG_ADD(LOG_UINT8, CustomM_CMD,     &CustomMotorCMD_Flag)
LOG_ADD(LOG_FLOAT, Plane_Angle,     &Plane_Angle_deg)

LOG_GROUP_STOP(Misc)
#endif