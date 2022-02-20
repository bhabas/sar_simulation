// CF HEADERS
#include "controller_gtc.h"



void controllerGTCInit(void)
{
    controllerGTCTest();
    X = nml_mat_new(3,1);
    initNN_Layers(&Scaler_Flip,W_flip,b_flip,NN_Params_Flip,3);
    initNN_Layers(&Scaler_Policy,W_policy,b_policy,NN_Params_Policy,3);
    controllerGTCReset();
    consolePrintf("GTC Initiated\n");
}

void controllerGTCReset(void)
{
    consolePrintf("GTC Reset\n");

    // RESET ERRORS
    e_PI = vzero();
    e_RI = vzero();

    // TURN POS/VEL CONTROLLER FLAGS ON
    kp_xf = 1.0;
    kd_xf = 1.0;

    // RESET SETPOINTS TO HOME POSITION
    x_d = mkvec(-0.25f,0.0f,0.4f);
    v_d = mkvec(0.0f,0.0f,0.0f);
    a_d = mkvec(0.0f,0.0f,0.0f);
    

    // RESET SYSTEM FLAGS
    tumbled = false;
    motorstop_flag = false;

    Moment_flag = false;
    policy_armed_flag = false;
    flip_flag = false;
    onceFlag = false;

    // RESET TRAJECTORY VALUES
    execute_traj = false;
    s_0_t = vzero();
    v_t = vzero();
    a_t = vzero();
    T = vzero();
    t_traj = vzero();

    // RESET LOGGED FLIP VALUES
    statePos_tr = vzero();
    stateVel_tr = vzero();
    stateQuat_tr = mkquat(0.0f,0.0f,0.0f,1.0f);
    stateOmega_tr = vzero();

    RREV_tr = 0.0;
    OF_x_tr = 0.0;
    OF_y_tr = 0.0;

    NN_tr_flip = 0.0;
    NN_tr_policy = 0.0;
    



}

bool controllerGTCTest(void)
{
    return true;
}

void GTC_Command(setpoint_t *setpoint)
{   
    switch(setpoint->cmd_type){
        case 0: // Reset
            controllerGTCReset();
            break;


        case 1: // Position
            x_d.x = setpoint->cmd_val1;
            x_d.y = setpoint->cmd_val2;
            x_d.z = setpoint->cmd_val3;
            kp_xf = setpoint->cmd_flag;
            break;


        case 2: // Velocity
            v_d.x = setpoint->cmd_val1;
            v_d.y = setpoint->cmd_val2;
            v_d.z = setpoint->cmd_val3;
            kd_xf = setpoint->cmd_flag;
            break;


        case 3: // Acceleration
            a_d.x = setpoint->cmd_val1;
            a_d.y = setpoint->cmd_val2;
            a_d.z = setpoint->cmd_val3;
            break;

        case 4: // Tumble-Detection
            tumble_detection = setpoint->cmd_flag;
            break;

        case 5: // Hard Set All Motorspeeds to Zero
            motorstop_flag = true;
            break;
        
        case 6: // Update Controller Gains
            
 
            break;

        case 7: // Execute Moment-Based Flip

            M_d.x = setpoint->cmd_val1*1e-3;
            M_d.y = setpoint->cmd_val2*1e-3;
            M_d.z = setpoint->cmd_val3*1e-3;

            Moment_flag = (bool)setpoint->cmd_flag;
            break;

        case 8: // Arm Policy Maneuver
            RREV_thr = setpoint->cmd_val1;
            G1 = setpoint->cmd_val2;
            G2 = setpoint->cmd_val3;

            policy_armed_flag = setpoint->cmd_flag;

            break;
            
        case 9: // Trajectory Values
            traj_type = (axis_direction)setpoint->cmd_flag;

            switch(traj_type){

                case x:

                    s_0_t.x = setpoint->cmd_val1;               // Starting position [m]
                    v_t.x = setpoint->cmd_val2;                 // Desired velocity [m/s]
                    a_t.x = setpoint->cmd_val3;                 // Max acceleration [m/s^2]
                    T.x = (a_t.x+fsqr(v_t.x))/(a_t.x*v_t.x);    // Find trajectory manuever length [s]

                    t_traj.x = 0.0f; // Reset timer
                    execute_traj = true;
                    break;

                case y:

                    s_0_t.y = setpoint->cmd_val1;
                    v_t.y = setpoint->cmd_val2;
                    a_t.y = setpoint->cmd_val3;
                    T.y = (a_t.y+fsqr(v_t.y))/(a_t.y*v_t.y); 

                    t_traj.y = 0.0f;
                    execute_traj = true;
                    break;

                case z:

                    s_0_t.z = setpoint->cmd_val1;
                    v_t.z = setpoint->cmd_val2;
                    a_t.z = setpoint->cmd_val3;
                    T.z = (a_t.z+fsqr(v_t.z))/(a_t.z*v_t.z); 

                    t_traj.z = 0.0f;
                    execute_traj = true;
                    break;
                

            }

            break;
    }
    
}

void controllerGTCTraj()
{
    for (int i = 0; i < 3; i++)
    {
    
        if(t_traj.idx[i]<=v_t.idx[i]/a_t.idx[i]) // t <= v/a
        {
            x_d.idx[i] = 0.5f*a_t.idx[i]*t_traj.idx[i]*t_traj.idx[i] + s_0_t.idx[i]; // 0.5*a*t^2 + s_0
            v_d.idx[i] = a_t.idx[i]*t_traj.idx[i]; // a*t
            a_d.idx[i] = a_t.idx[i]; // a

        }

        // CONSTANT VELOCITY TRAJECTORY
        if(v_t.idx[i]/a_t.idx[i] < t_traj.idx[i]) // v/a < t
        {
            x_d.idx[i] = v_t.idx[i]*t_traj.idx[i] - fsqr(v_t.idx[i])/(2.0f*a_t.idx[i]) + s_0_t.idx[i]; // v*t - (v/(2*a))^2 + s_0
            v_d.idx[i] = v_t.idx[i]; // v
            a_d.idx[i] = 0.0f;
        }

        // // POINT-TO-POINT (1m) POSITION TRAJECTORY
        // if(v_t.idx[i]/a_t.idx[i] <= t_traj.idx[i] && t_traj.idx[i] < (T.idx[i]-v_t.idx[i]/a_t.idx[i])) // v/a < t < (T-v/a)
        // {
        //     x_d.idx[i] = v_t.idx[i]*t_traj.idx[i] - fsqr(v_t.idx[i])/(2.0f*a_t.idx[i]) + s_0_t.idx[i]; // v*t - (v/2*a)^2 = s_0
        //     v_d.idx[i] = v_t.idx[i]; // v
        //     a_d.idx[i] = 0.0f; 
        // }

        // if((T.idx[i]-v_t.idx[i]/a_t.idx[i]) < t_traj.idx[i] && t_traj.idx[i] <= T.idx[i]) // (T-v/a) < t < T
        // {
        //     x_d.idx[i] = (2.0f*a_t.idx[i]*v_t.idx[i]*T.idx[i]-2.0f*fsqr(v_t.idx[i])-fsqr(a_t.idx[i])*fsqr(t_traj.idx[i]-T.idx[i]))/(2.0f*a_t.idx[i]) + s_0_t.idx[i];
        //     v_d.idx[i] = a_t.idx[i]*(T.idx[i]-t_traj.idx[i]);
        //     a_d.idx[i] = -a_t.idx[i];
        // }


        t_traj.idx[i] += dt;
    }
    

    
}

void controllerGTC(control_t *control, setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick)
{
    if (RATE_DO_EXECUTE(RATE_500_HZ, tick)) {

        if (setpoint->GTC_cmd_rec == true)
            {
                GTC_Command(setpoint);
                setpoint->GTC_cmd_rec = false;
            }

        if (errorReset){
            controllerGTCReset();
            errorReset = false;
            }

        if(execute_traj){
            controllerGTCTraj();
        }

        // SYSTEM PARAMETERS 
        J = mdiag(1.65717e-5f, 1.66556e-5f, 2.92617e-5f); // Rotational Inertia of CF [kg m^2]

        // CONTROL GAINS
        Kp_p = mkvec(P_kp_xy,P_kp_xy,P_kp_z);
        Kd_p = mkvec(P_kd_xy,P_kd_xy,P_kd_z);
        Ki_p = mkvec(P_ki_xy,P_ki_xy,P_ki_z);

        Kp_R = mkvec(R_kp_xy,R_kp_xy,R_kp_z);
        Kd_R = mkvec(R_kd_xy,R_kd_xy,R_kd_z);
        Ki_R = mkvec(R_ki_xy,R_ki_xy,R_ki_z);

        // =========== STATE DEFINITIONS =========== //
        statePos = mkvec(state->position.x, state->position.y, state->position.z);                      // [m]
        stateVel = mkvec(state->velocity.x, state->velocity.y, state->velocity.z);                      // [m]
        stateOmega = mkvec(radians(sensors->gyro.x), radians(sensors->gyro.y), radians(sensors->gyro.z));   // [rad/s]
        stateQuat = mkquat(state->attitudeQuaternion.x,
                        state->attitudeQuaternion.y,
                        state->attitudeQuaternion.z,
                        state->attitudeQuaternion.w);

        // EULER ANGLES EXPRESSED IN YZX NOTATION
        stateEul = quat2eul(stateQuat);
        stateEul.x = degrees(stateEul.x);
        stateEul.y = degrees(stateEul.y);
        stateEul.z = degrees(stateEul.z);

        RREV = stateVel.z/(h_ceiling - statePos.z);
        OF_x = stateVel.y/(h_ceiling - statePos.z);
        OF_y = stateVel.x/(h_ceiling - statePos.z);
        d_ceil = (h_ceiling - statePos.z);

        X->data[0][0] = 4.7f;
        X->data[1][0] = -5.0f;
        X->data[2][0] = 0.4f; // d_ceiling [m]

        if(statePos.z >= 2.3)
        {
            motorstop_flag = true;
        }


        // =========== STATE SETPOINTS =========== //
        // x_d = mkvec(setpoint->position.x,setpoint->position.y,setpoint->position.z);             // Pos-desired [m]
        // v_d = mkvec(setpoint->velocity.x, setpoint->velocity.y, setpoint->velocity.z);             // Vel-desired [m/s]
        // a_d = mkvec(setpoint->acceleration.x, setpoint->acceleration.y, setpoint->acceleration.z); // Acc-desired [m/s^2]

        omega_d = mkvec(0.0f,0.0f,0.0f);    // Omega-desired [rad/s]
        domega_d = mkvec(0.0f,0.0f,0.0f);   // Omega-Accl. [rad/s^2]

        eul_d = mkvec(0.0f,0.0f,0.0f);
        quat_d = rpy2quat(eul_d);           // Desired orientation from eul angles [ZYX NOTATION]

        // =========== ROTATION MATRIX =========== //
        // R changes Body axes to be in terms of Global axes
        // https://www.andre-gaschler.com/rotationconverter/
        R = quat2rotmat(stateQuat); // Quaternion to Rotation Matrix Conversion
        b3 = mvmul(R, e_3);         // Current body vertical axis in terms of global axes | [b3 = R*e_3] 
        
        // TUMBLE DETECTION
        if (b3.z <= 0 && tumble_detection == true){
            tumbled = true;
        }

        // =========== TRANSLATIONAL EFFORT =========== //
        e_x = vsub(statePos, x_d); // [e_x = pos-x_d]
        e_v = vsub(stateVel, v_d); // [e_v = vel-v_d]


        // POS. INTEGRAL ERROR
        e_PI.x += (e_x.x)*dt;
        e_PI.x = clamp(e_PI.x, -i_range_xy, i_range_xy);

        e_PI.y += (e_x.y)*dt;
        e_PI.y = clamp(e_PI.y, -i_range_xy, i_range_xy);

        e_PI.z += (e_x.z)*dt;
        e_PI.z = clamp(e_PI.z, -i_range_z, i_range_z);

        /* [F_thrust_ideal = -kp_x*e_x*(kp_x_flag) + -kd_x*e_v + -kI_x*e_PI*(kp_x_flag) + m*g*e_3 + m*a_d] */
        temp1_v = veltmul(vneg(Kp_p), e_x);
        temp1_v = vscl(kp_xf,temp1_v);
        temp2_v = veltmul(vneg(Kd_p), e_v);
        temp1_v = vscl(kd_xf,temp1_v);
        temp3_v = veltmul(vneg(Ki_p), e_PI);
        P_effort = vadd3(temp1_v,temp2_v,temp3_v);
        temp1_v = vscl(m*g, e_3); // Feed-forward term
        temp2_v = vscl(m, a_d);

        F_thrust_ideal = vadd3(P_effort, temp1_v,temp2_v); 

        // =========== DESIRED BODY AXES =========== // 
        b3_d = vnormalize(F_thrust_ideal);
        b2_d = vnormalize(vcross(b3_d, b1_d));      // [b3_d x b1_d] | body-fixed horizontal axis
        temp1_v = vnormalize(vcross(b2_d, b3_d));
        R_d = mcolumns(temp1_v, b2_d, b3_d);        // Desired rotation matrix from calculations


        // ATTITUDE CONTROL
        if (attCtrlEnable){ 
            R_d = quat2rotmat(quat_d); // Desired rotation matrix from att. control
        }


        // =========== ROTATIONAL ERRORS =========== // 
        RdT_R = mmul(mtranspose(R_d), R);       // [R_d'*R]
        RT_Rd = mmul(mtranspose(R), R_d);       // [R'*R_d]

        temp1_v = dehat(msub(RdT_R, RT_Rd));    // [dehat(R_d'*R - R'*R)]
        e_R = vscl(0.5f, temp1_v);              // Rotation error | [eR = 0.5*dehat(R_d'*R - R'*R)]

        temp1_v = mvmul(RT_Rd, omega_d);        // [R'*R_d*omega_d]
        e_w = vsub(stateOmega, temp1_v);        // Ang. vel error | [e_w = omega - R'*R_d*omega_d] 

        // ROT. INTEGRAL ERROR
        e_RI.x += (e_R.x)*dt;
        e_RI.x = clamp(e_RI.x, -i_range_R_xy, i_range_R_xy);

        e_RI.y += (e_R.y)*dt;
        e_RI.y = clamp(e_RI.y, -i_range_R_xy, i_range_R_xy);

        e_RI.z += (e_R.z)*dt;
        e_RI.z = clamp(e_RI.z, -i_range_R_z, i_range_R_z);

        // =========== CONTROL EQUATIONS =========== // 
        /* [M = -kp_R*e_R - kd_R*e_w -ki_R*e_RI + Gyro_dyn] */

        temp1_v = veltmul(vneg(Kp_R), e_R);     // [-kp_R*e_R]
        temp2_v = veltmul(vneg(Kd_R), e_w);     // [-kd_R*e_w]
        temp3_v = veltmul(vneg(Ki_R), e_RI);    // [-ki_R*e_RI]
        R_effort = vadd3(temp1_v,temp2_v,temp3_v);

        /* Gyro_dyn = [omega x (J*omega)] - [J*( hat(omega)*R'*R_d*omega_d - R'*R_d*domega_d )] */
        temp1_v = vcross(stateOmega, mvmul(J, stateOmega)); // [omega x J*omega]


        temp1_m = mmul(hat(stateOmega), RT_Rd); //  hat(omega)*R'*R_d
        temp2_v = mvmul(temp1_m, omega_d);      // (hat(omega)*R'*R_d)*omega_d
        temp3_v = mvmul(RT_Rd, domega_d);       // (R'*R_d*domega_d)

        temp4_v = mvmul(J, vsub(temp2_v, temp3_v)); // J*(hat(omega)*R'*R_d*omega_d - R'*R_d*domega_d)
        Gyro_dyn = vsub(temp1_v,temp4_v);

        F_thrust = vdot(F_thrust_ideal, b3);    // Project ideal thrust onto b3 vector [N]
        M = vadd(R_effort,Gyro_dyn);            // Control moments [Nm]
    
        // =========== THRUST AND MOMENTS [FORCE NOTATION] =========== // 
        if(!tumbled){
            if(policy_armed_flag == true){ 
                
                switch(POLICY_TYPE)
                {
                    case RL:
                    {
                        if(RREV >= RREV_thr && onceFlag == false){
                            onceFlag = true;
                            flip_flag = true;  

                            // UPDATE AND RECORD FLIP VALUES
                            statePos_tr = statePos;
                            stateVel_tr = stateVel;
                            stateQuat_tr = stateQuat;
                            stateOmega_tr = stateOmega;

                            OF_y_tr = OF_y;
                            OF_x_tr = OF_x;
                            RREV_tr = RREV;
                        
                            M_d.x = 0.0f;
                            M_d.y = -G1*1e-3;
                            M_d.z = 0.0f;
                        }
                        break;
                    }

                    case NN:
                    {   
                        // NN_flip = NN_Forward_Flip(X,&Scaler_Flip,W_flip,b_flip);
                        // NN_policy = -NN_Forward_Policy(X,&Scaler_Policy,W_policy,b_policy);

                        // DEBUG_PRINT("NN_Flip: %.5f \t NN_Policy: %.5f \n",NN_flip,NN_policy);

                        if(NN_flip >= 0.9 && onceFlag == false)
                        {   
                            onceFlag = true;
                            flip_flag = true;

                            // UPDATE AND RECORD FLIP VALUES
                            statePos_tr = statePos;
                            stateVel_tr = stateVel;
                            stateQuat_tr = stateQuat;
                            stateOmega_tr = stateOmega;

                            OF_y_tr = OF_y;
                            OF_x_tr = OF_x;
                            RREV_tr = RREV;
                            d_ceil_tr = d_ceil;

                            NN_tr_flip = NN_Forward_Flip(X,&Scaler_Flip,W_flip,b_flip);
                            NN_tr_policy = NN_Forward_Policy(X,&Scaler_Policy,W_policy,b_policy);


                            M_d.x = 0.0f;
                            M_d.y = -NN_tr_policy*1e-3;
                            M_d.z = 0.0f;
                        }

                        break;
                    }

                }

                
                if(flip_flag == true)
                {
                    // Doubling front motor values and zeroing back motors is
                    // equal to increasing front motors and decreasing back motors.
                    // This gives us the highest possible moment and avoids PWM cutoff issue
                    M = vscl(2.0f,M_d); 
                    F_thrust = 0.0f;
                }

            }

            // MANUAL MOMENT CONTROL
            else if (Moment_flag == true){ 

                M.x = M_d.x;
                M.y = M_d.y;
                M.z = M_d.z;

                M = vscl(2.0f,M_d); // Need to double moment to ensure it survives the PWM<0 cutoff
                F_thrust = 0.0f;

            }
            else{ // Behave like normal
                F_thrust = F_thrust;
                M = M;
            }

        }
        else if(tumbled && tumble_detection == true){
            // consolePrintf("System Tumbled: \n");
            F_thrust = 0.0f;
            M.x = 0.0f;
            M.y = 0.0f;
            M.z = 0.0f;
        }

        
        // =========== CONVERT THRUSTS AND MOMENTS TO PWM =========== // 
        f_thrust_g = F_thrust/4.0f*Newton2g;
        f_roll_g = M.x/(4.0f*dp)*Newton2g;
        f_pitch_g = M.y/(4.0f*dp)*Newton2g;
        f_yaw_g = M.z/(4.0*c_tf)*Newton2g;

        f_thrust_g = clamp(f_thrust_g,0.0,f_MAX*0.8);    // Clamp thrust to prevent control saturation

        // Add respective thrust components and limit to (0 <= PWM <= 60,000)
        M1_pwm = limitPWM(thrust2PWM(f_thrust_g + f_roll_g - f_pitch_g + f_yaw_g)); 
        M2_pwm = limitPWM(thrust2PWM(f_thrust_g + f_roll_g + f_pitch_g - f_yaw_g));
        M3_pwm = limitPWM(thrust2PWM(f_thrust_g - f_roll_g + f_pitch_g + f_yaw_g));
        M4_pwm = limitPWM(thrust2PWM(f_thrust_g - f_roll_g - f_pitch_g - f_yaw_g));

        if(motorstop_flag){ // Cutoff all motor values
            f_thrust_g = 0.0f;
            M1_pwm = 0;
            M2_pwm = 0;
            M3_pwm = 0;
            M4_pwm = 0;
        }


        // THESE CONNECT TO POWER_DISTRIBUTION_STOCK.C TO 
        control->thrust = f_thrust_g;
        control->roll = (int16_t)(f_roll_g*1e3f);
        control->pitch = (int16_t)(f_pitch_g*1e3f);
        control->yaw = (int16_t)(f_yaw_g*1e3f);


        

        // compressStates();
        // compressSetpoints();
        // compressFlipStates();
        

    }
    
    if(safeModeEnable) // If safeMode is enabled then override all PWM commands
    {
        M1_pwm = 0;
        M2_pwm = 0;
        M3_pwm = 0;
        M4_pwm = 0;
    }
    else
    {
        // motorsSetRatio(MOTOR_M1, M4_pwm);
        // motorsSetRatio(MOTOR_M2, M3_pwm);
        // motorsSetRatio(MOTOR_M3, M2_pwm);
        // motorsSetRatio(MOTOR_M4, M1_pwm);
    }

    

}

// // PARAMETER GROUPS
// PARAM_GROUP_START(GTC_Params)
// PARAM_ADD(PARAM_FLOAT, P_kp_xy, &P_kp_xy)
// PARAM_ADD(PARAM_FLOAT, P_kp_z,  &P_kp_z)
// PARAM_ADD(PARAM_FLOAT, P_kd_xy, &P_kd_xy) 
// PARAM_ADD(PARAM_FLOAT, i_range_xy, &i_range_xy)
// PARAM_ADD(PARAM_FLOAT, P_kd_z,  &P_kd_z)
// PARAM_ADD(PARAM_FLOAT, P_ki_xy, &P_ki_xy)
// PARAM_ADD(PARAM_FLOAT, P_ki_z,  &P_ki_z)
// PARAM_ADD(PARAM_FLOAT, i_range_z, &i_range_z)

// PARAM_ADD(PARAM_FLOAT, R_kp_xy, &R_kp_xy)
// PARAM_ADD(PARAM_FLOAT, R_kd_xy, &R_kd_xy) 
// PARAM_ADD(PARAM_FLOAT, R_ki_xy, &R_ki_xy)
// PARAM_ADD(PARAM_FLOAT, i_range_R_xy, &i_range_R_xy)

// PARAM_ADD(PARAM_FLOAT, R_kp_z,  &R_kp_z)
// PARAM_ADD(PARAM_FLOAT, R_kd_z,  &R_kd_z)
// PARAM_ADD(PARAM_FLOAT, R_ki_z,  &R_ki_z)
// PARAM_ADD(PARAM_FLOAT, i_range_R_z, &i_range_R_z)

// PARAM_ADD(PARAM_FLOAT, b1_d_x, &b1_d.x)
// PARAM_ADD(PARAM_FLOAT, b1_d_y, &b1_d.y)
// PARAM_ADD(PARAM_FLOAT, b1_d_z, &b1_d.z)

// PARAM_ADD(PARAM_FLOAT, CF_mass, &m)

// // PARAM_ADD(PARAM_UINT8, AttCtrl, &attCtrlEnable)
// // PARAM_ADD(PARAM_UINT8, Tumbled, &tumbled)
// // PARAM_ADD(PARAM_UINT8, Error_Reset, &errorReset)
// PARAM_ADD(PARAM_UINT8, SafeModeFlag, &safeModeEnable)

// PARAM_GROUP_STOP(GTC_Params)


// // LOGGING GROUPS

// LOG_GROUP_START(LogStateData_GTC)
// LOG_ADD(LOG_UINT32, Z_xy,   &StatesZ_GTC.xy)
// LOG_ADD(LOG_INT16,  Z_z,    &StatesZ_GTC.z)

// LOG_ADD(LOG_UINT32, Z_vxy,  &StatesZ_GTC.vxy)
// LOG_ADD(LOG_INT16,  Z_vz,   &StatesZ_GTC.vz)

// LOG_ADD(LOG_UINT32, Z_quat, &StatesZ_GTC.quat)

// LOG_ADD(LOG_UINT32, Z_wxy,  &StatesZ_GTC.wxy)
// LOG_ADD(LOG_INT16,  Z_wz,   &StatesZ_GTC.wz)

// LOG_ADD(LOG_UINT32, Z_OFxy, &StatesZ_GTC.OF_xy)
// LOG_ADD(LOG_INT16,  Z_RREV, &StatesZ_GTC.RREV)
// LOG_ADD(LOG_INT16,  Z_d_ceil, &StatesZ_GTC.d_ceil)

// LOG_ADD(LOG_UINT32, Z_FMz, &StatesZ_GTC.FMz)
// LOG_ADD(LOG_UINT32, Z_Mxy, &StatesZ_GTC.Mxy)

// LOG_ADD(LOG_UINT32, Z_PWM12, &StatesZ_GTC.MS_PWM12)
// LOG_ADD(LOG_UINT32, Z_PWM34, &StatesZ_GTC.MS_PWM34)

// LOG_ADD(LOG_UINT32, Z_NN_FP, &StatesZ_GTC.NN_FP)
// LOG_GROUP_STOP(LogStateData_GTC)



// LOG_GROUP_START(LogMiscData_GTC)
// LOG_ADD(LOG_UINT8, Flip_Flag, &flip_flag)
// // LOG_ADD(LOG_UINT8, Tumbled_Flag, &tumbled)
// // LOG_ADD(LOG_UINT8, Tumbled_Det_Flag, &tumble_detection)
// // LOG_ADD(LOG_UINT8, MotorStop_Flag, &motorstop_flag)
// // LOG_ADD(LOG_UINT8, Execute_Traj_Flag, &execute_traj)
// // LOG_ADD(LOG_UINT8, Policy_Armed_Flag, &policy_armed_flag)
// // LOG_ADD(LOG_UINT8, Moment_Flag, &Moment_flag)
// // LOG_ADD(LOG_UINT8, Att_Ctrl_Flag, &attCtrlEnable)
// LOG_GROUP_STOP(LogMiscData_GTC)



// LOG_GROUP_START(LogSetPoints_GTC)
// LOG_ADD(LOG_UINT32, Z_xy,   &setpointZ_GTC.xy)
// LOG_ADD(LOG_INT16,  Z_z,    &setpointZ_GTC.z)

// LOG_ADD(LOG_UINT32, Z_vxy,  &setpointZ_GTC.vxy)
// LOG_ADD(LOG_INT16,  Z_vz,   &setpointZ_GTC.vz)

// LOG_ADD(LOG_UINT32, Z_axy,  &setpointZ_GTC.axy)
// LOG_ADD(LOG_INT16,  Z_az,   &setpointZ_GTC.az)
// LOG_GROUP_STOP(LogSetPoints_GTC)


// LOG_GROUP_START(LogFlipData_GTC)
// LOG_ADD(LOG_UINT32, Z_xy,   &FlipStatesZ_GTC.xy)
// LOG_ADD(LOG_INT16,  Z_z,    &FlipStatesZ_GTC.z)

// LOG_ADD(LOG_UINT32, Z_vxy,  &FlipStatesZ_GTC.vxy)
// LOG_ADD(LOG_INT16,  Z_vz,   &FlipStatesZ_GTC.vz)

// LOG_ADD(LOG_UINT32, Z_quat, &FlipStatesZ_GTC.quat)

// LOG_ADD(LOG_UINT32, Z_wxy,  &FlipStatesZ_GTC.wxy)
// LOG_ADD(LOG_INT16,  Z_wz,   &FlipStatesZ_GTC.wz)

// LOG_ADD(LOG_UINT32, Z_OFxy, &FlipStatesZ_GTC.OF_xy)
// LOG_ADD(LOG_INT16,  Z_RREV, &FlipStatesZ_GTC.RREV)
// LOG_ADD(LOG_INT16,  Z_d_ceil, &FlipStatesZ_GTC.d_ceil)

// LOG_GROUP_STOP(LogFlipData_GTC)
