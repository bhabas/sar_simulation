
#include "controller.h"
#include <unistd.h>


void controllerGTCInit(void)
{
    controllerGTCTest();
    initScaler(&Scaler_Flip,str1);
    initScaler(&Scaler_Policy,str2);
    initNN_Layers(W_policy,b_policy,path_policy,3);
    initNN_Layers(W_flip,b_flip,path_flip,2);
    // controllerGTCReset(_CTRL);
    printf("GTC Initiated\n");
}

void controllerGTCReset(Controller* _CTRL)
{
    // printf("GTC Reset\n");
    // Reset errors to zero
    e_PI = vzero();
    e_RI = vzero();

    kp_xf = 1.0;
    kd_xf = 1.0;


    x_d = mkvec(0.0f,0.0f,0.4f);
    v_d = mkvec(0.0f,0.0f,0.0f);
    a_d = mkvec(0.0f,0.0f,0.0f);
    

    tumbled = false;
    motorstop_flag = false;

    Moment_flag = false;
    policy_armed_flag = false;
    flip_flag = false;
    onceFlag = false;


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




    // ROS SPECIFIC VALUES
    // ==========================
    _CTRL->_impact_flag = false;
    _CTRL->_slowdown_type = 0;
    _CTRL->adjustSimSpeed(_CTRL->_SIM_SPEED);




}

bool controllerGTCTest(void)
{
    return true;
}

void GTC_Command(setpoint_t *setpoint, Controller* _CTRL)
{   
    // printf("cmd_type: %d\n cmd_1: %.3f\t cmd_2: %.3f\t cmd_3: %.3f\n cmd_flag: %.3f\n",
    //     setpoint->cmd_type,
    //     setpoint->cmd_val1,
    //     setpoint->cmd_val2,
    //     setpoint->cmd_val3,
    //     setpoint->cmd_flag);

    switch(setpoint->cmd_type){
        case 0: // Reset
            controllerGTCReset(_CTRL);
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
            
            ros::param::get("P_kp_xy",P_kp_xy);
            ros::param::get("P_kd_xy",P_kd_xy);
            ros::param::get("P_ki_xy",P_ki_xy);
            ros::param::get("i_range_xy",i_range_xy);

            ros::param::get("P_kp_z",P_kp_z);
            ros::param::get("P_kd_z",P_kd_z);
            ros::param::get("P_ki_z",P_ki_z);
            ros::param::get("i_range_z",i_range_z);

            ros::param::get("R_kp_xy",R_kp_xy);
            ros::param::get("R_kd_xy",R_kd_xy);
            ros::param::get("R_ki_xy",R_ki_xy);
            ros::param::get("i_range_R_xy",i_range_R_xy);
            
            ros::param::get("R_kp_z",R_kp_z);
            ros::param::get("R_kd_z",R_kd_z);
            ros::param::get("R_ki_z",R_ki_z);
            ros::param::get("i_range_R_z",i_range_R_z);
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
                                         const uint32_t tick,
                                         Controller* _CTRL) 
{
    if (RATE_DO_EXECUTE(RATE_500_HZ, tick)) {

        if (setpoint->GTC_cmd_rec == true)
            {
                GTC_Command(setpoint, _CTRL);
                setpoint->GTC_cmd_rec = false;
            }

        if (errorReset){
            controllerGTCReset(_CTRL);
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


        RREV = stateVel.z/(h_ceiling - statePos.z);
        OF_x = stateVel.y/(h_ceiling - statePos.z);
        OF_y = stateVel.x/(h_ceiling - statePos.z);

        X->data[0][0] = RREV;
        X->data[1][0] = OF_y;
        X->data[2][0] = (h_ceiling - statePos.z); // d_ceiling [m]

        


        // =========== STATE SETPOINTS =========== //
        // x_d = mkvec(setpoint->position.x,setpoint->position.y,setpoint->position.z);             // Pos-desired [m]
        // v_d = mkvec(setpoint->velocity.x, setpoint->velocity.y, setpoint->velocity.z);             // Vel-desired [m/s]
        // a_d = mkvec(setpoint->acceleration.x, setpoint->acceleration.y, setpoint->acceleration.z); // Acc-desired [m/s^2]

        // =========== STATE SETPOINTS =========== //
    
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
                            _CTRL->_t_flip = ros::Time::now();
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
                        _CTRL->_flip_NN = NN_Flip(X,&Scaler_Flip,W_flip,b_flip);

                        if(_CTRL->_flip_NN >= 0.5 && onceFlag == false)
                        {   
                            onceFlag = true;
                            flip_flag = true;

                            // UPDATE AND RECORD FLIP VALUES
                            _CTRL->_t_flip = ros::Time::now();
                            statePos_tr = statePos;
                            stateVel_tr = stateVel;
                            stateQuat_tr = stateQuat;
                            stateOmega_tr = stateOmega;

                            OF_y_tr = OF_y;
                            OF_x_tr = OF_x;
                            RREV_tr = RREV;

                            _CTRL->_policy_NN = -NN_Policy(X,&Scaler_Policy,W_policy,b_policy);
                            M_d.x = 0.0f;
                            M_d.y = _CTRL->_policy_NN;
                            M_d.z = 0.0f;
                        }

                        break;
                    }

                }

                
                if(flip_flag == true)
                {
                
                    // My->Front rotors(+) & Rear rotors(-) = My->Double Front rotors(+) effort
                    M = vscl(2.0f,M_d); 
                    F_thrust = 0.0f;

                    // RECORD MOTOR THRUST TYPES AT FLIP

                    F_thrus_t_flip = F_thrust;
                    M_x_flip = M.x/2.0f;
                    M_y_flip = M.y/2.0f;
                    M_z_flip = M.z/2.0f;
                }

                

            }
            else if (Moment_flag == true){

                M.x = M_d.x;
                M.y = M_d.y;
                M.z = M_d.z;

                M = vscl(2.0f,M_d); // Need to double moment to ensure it survives the PWM<0 cutoff
                F_thrust = 0.0f;

            }
            else{
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

        if(motorstop_flag){ // Cutoff all motor values
            f_thrust_g = 0.0f;
            f_roll_g = 0.0f;
            f_pitch_g = 0.0f;
            f_yaw_g = 0.0f;
        }

        
        M1_pwm = limitPWM(thrust2PWM(f_thrust_g + f_roll_g - f_pitch_g + f_yaw_g)); // Add respective thrust components and limit to (0 <= PWM <= 60,000)
        M2_pwm = limitPWM(thrust2PWM(f_thrust_g + f_roll_g + f_pitch_g - f_yaw_g));
        M3_pwm = limitPWM(thrust2PWM(f_thrust_g - f_roll_g + f_pitch_g + f_yaw_g));
        M4_pwm = limitPWM(thrust2PWM(f_thrust_g - f_roll_g - f_pitch_g - f_yaw_g));

        MS1 = (uint16_t)sqrt(PWM2thrust(M1_pwm)*g2Newton/kf);
        MS2 = (uint16_t)sqrt(PWM2thrust(M2_pwm)*g2Newton/kf);
        MS3 = (uint16_t)sqrt(PWM2thrust(M3_pwm)*g2Newton/kf);
        MS4 = (uint16_t)sqrt(PWM2thrust(M4_pwm)*g2Newton/kf);


        
        // ============================
        //          C++ CODE
        // ============================
       

        _CTRL->_MS_msg.MotorSpeeds = {MS1,MS2,MS3,MS4};
        _CTRL->_MS_Publisher.publish(_CTRL->_MS_msg);

        
        // SIMULATION SLOWDOWN
        if(_CTRL->_LANDING_SLOWDOWN_FLAG==true){

            // WHEN CLOSE TO THE CEILING REDUCE SIM SPEED
            if(_CTRL->_H_CEILING-statePos.z<=0.5 && _CTRL->_slowdown_type == 0){
                
                _CTRL->adjustSimSpeed(_CTRL->_SIM_SLOWDOWN_SPEED);
                _CTRL->_slowdown_type = 1;

            }

            // IF IMPACTED OR MISSED CEILING, INCREASE SIM SPEED TO DEFAULT
            if(_CTRL->_impact_flag == true && _CTRL->_slowdown_type == 1)
            {
                _CTRL->adjustSimSpeed(_CTRL->_SIM_SPEED);
                _CTRL->_slowdown_type = 2;
            }
            else if(stateVel.z <= -0.5 && _CTRL->_slowdown_type == 1){
                _CTRL->adjustSimSpeed(_CTRL->_SIM_SPEED);
                _CTRL->_slowdown_type = 2;
            }

        }



        _CTRL->_ctrl_msg.RREV = RREV;
        _CTRL->_ctrl_msg.OF_y = OF_y;
        _CTRL->_ctrl_msg.OF_x = OF_x;

        // FLIP INFO
        _CTRL->_ctrl_msg.flip_flag = flip_flag;
        _CTRL->_ctrl_msg.RREV_tr = RREV_tr;
        _CTRL->_ctrl_msg.OF_x_tr = OF_x_tr;
        _CTRL->_ctrl_msg.OF_y_tr = OF_y_tr;
        _CTRL->_ctrl_msg.FM_flip = {F_thrus_t_flip,M_x_flip*1.0e3,M_y_flip*1.0e3,M_z_flip*1.0e3};

        _CTRL->_ctrl_msg.policy_NN = _CTRL->_policy_NN;
        _CTRL->_ctrl_msg.flip_NN = _CTRL->_flip_NN;


        _CTRL->_ctrl_msg.Pose_tr.header.stamp = _CTRL->_t_flip;             

        _CTRL->_ctrl_msg.Pose_tr.pose.position.x = statePos_tr.x;
        _CTRL->_ctrl_msg.Pose_tr.pose.position.y = statePos_tr.y;
        _CTRL->_ctrl_msg.Pose_tr.pose.position.z = statePos_tr.z;

        _CTRL->_ctrl_msg.Pose_tr.pose.orientation.x = stateQuat_tr.x;
        _CTRL->_ctrl_msg.Pose_tr.pose.orientation.y = stateQuat_tr.y;
        _CTRL->_ctrl_msg.Pose_tr.pose.orientation.z = stateQuat_tr.z;
        _CTRL->_ctrl_msg.Pose_tr.pose.orientation.w = stateQuat_tr.w;

        _CTRL->_ctrl_msg.Twist_tr.linear.x = stateVel_tr.x;
        _CTRL->_ctrl_msg.Twist_tr.linear.y = stateVel_tr.y;
        _CTRL->_ctrl_msg.Twist_tr.linear.z = stateVel_tr.z;

        _CTRL->_ctrl_msg.Twist_tr.angular.x = stateOmega_tr.x;
        _CTRL->_ctrl_msg.Twist_tr.angular.y = stateOmega_tr.y;
        _CTRL->_ctrl_msg.Twist_tr.angular.z = stateOmega_tr.z;

        _CTRL->_ctrl_msg.FM = {F_thrust,M.x*1.0e3,M.y*1.0e3,M.z*1.0e3};
        _CTRL->_ctrl_msg.MS_PWM = {M1_pwm,M2_pwm,M3_pwm,M4_pwm};
        _CTRL->_CTRL_Publisher.publish(_CTRL->_ctrl_msg);

        // DATA HANDLING
        if (tick%50 == 0){ // General Debugging output
        cout << fixed;
        cout << setprecision(4) << endl <<
        "t: " << _CTRL->_t << "\tCmd: "  << endl << 
        endl <<

        "RREV_thr: " << RREV_thr << "\tG1: " << G1 << "\tG2: " << G2 << endl << 
        "RREV: " << RREV << "\tOF_x: " << OF_x << "\tOF_y: " << OF_y << endl <<
        "RREV_flip: " << RREV_tr << "\tOF_x_tr: " << OF_x_tr << "\tOF_y_tr: " << OF_y_tr << endl << 
        endl << 

        "Kp_P: " << Kp_p.x << "  " << Kp_p.y << "  " << Kp_p.z << "\t" <<
        "Kp_R: " << Kp_R.x << "  " << Kp_R.y << "  " << Kp_R.z << endl <<
        "Kd_P: " << Kd_p.x << "  " << Kd_p.y << "  " << Kd_p.z << "\t" <<
        "Kd_R: " << Kd_R.x << "  " << Kd_R.y << "  " << Kd_R.z << endl <<
        "Ki_P: " << Ki_p.x << "  " << Ki_p.y << "  " << Ki_p.z << "\t" <<
        "Ki_R: " << Ki_R.x << "  " << Ki_R.y << "  " << Ki_R.z << endl <<
        endl <<

        setprecision(0) <<
        "Policy_armed: " << policy_armed_flag <<  "\tFlip_flag: " << flip_flag << "\t_impact_flag: " << _CTRL->_impact_flag << endl <<
        "Tumble Detection: " << tumble_detection << "\t\tTumbled: " << tumbled << endl <<
        "kp_xf: " << kp_xf << " \tkd_xf: " << kp_xf << endl <<
        "slowdown_type: " << _CTRL->_slowdown_type << endl << 
        endl <<
        
        setprecision(3) <<
        "x_d: " << x_d.x << "  " << x_d.y << "  " << x_d.z << endl <<
        "v_d: " << v_d.x << "  " << v_d.y << "  " << v_d.z << endl <<
        "a_d: " << a_d.x << "  " << a_d.y << "  " << a_d.z << endl <<
        endl << 

        setprecision(3) <<
        "Pos [m]: " << statePos.x << "  " << statePos.y << "  " << statePos.z << "\t" <<
        "e_x: " << e_x.x << "  " << e_x.y << "  " << e_x.z << endl <<
        "Vel [m/s]: " << stateVel.x << "  " << stateVel.y << "  " << stateVel.z << "\t" <<
        "e_v: " << e_v.x << "  " << e_v.y << "  " << e_v.z << endl <<
        "Omega [rad/s]: " << stateOmega.x << "  " << stateOmega.y << "  " << stateOmega.z << "\t" <<
        "e_w: " << e_w.x << "  " << e_w.y << "  " << e_w.z << endl <<

        "e_PI : " << e_PI.x << "  " << e_PI.y << "  " << e_PI.z << "\t" <<
        "e_RI: " << e_RI.x << "  " << e_RI.y << "  " << e_RI.z << endl <<
        endl << 

        "R:\n" << 
        R.m[0][0] << "  " << R.m[0][1] << "  " << R.m[0][2] << "\n" <<
        R.m[1][0] << "  " << R.m[1][1] << "  " << R.m[1][2] << "\n" <<
        R.m[2][0] << "  " << R.m[2][1] << "  " << R.m[2][2] << "\n" <<
        endl <<

        "R_d:\n" << 
        R_d.m[0][0] << "  " << R_d.m[0][1] << "  " << R_d.m[0][2] << "\n" <<
        R_d.m[1][0] << "  " << R_d.m[1][1] << "  " << R_d.m[1][2] << "\n" <<
        R_d.m[2][0] << "  " << R_d.m[2][1] << "  " << R_d.m[2][2] << "\n" <<
        endl <<
        
        "e_R : " << e_R.x << "  " << e_R.y << "  " << e_R.z << "\t" <<
        "e_R (deg): " << e_R.x*180.0f/M_PI << "  " << e_R.y*180.0f/M_PI << "  " << e_R.z*180.0f/M_PI << endl <<
        endl <<

        "FM [N/N*mm]: " << F_thrust << "  " << M.x*1.0e3 << "  " << M.y*1.0e3 << "  " << M.z*1.0e3 << endl <<
        "f [g]: " << f_thrust_g << "  " << f_roll_g << "  " << f_pitch_g << "  " << f_yaw_g << "  " << "\t" << endl << 
        endl <<
        
        setprecision(0) <<
        "MS_PWM: " << M1_pwm << "  " << M2_pwm << "  " << M3_pwm << "  " << M4_pwm << endl <<
        "MS: " << MS1 << "  " << MS2 << "  " << MS3 << "  " << MS4 << endl <<
        endl << 

        "=============== " << endl; 
        }

    }
    
}



int main(int argc, char **argv)
{   
    ros::init(argc, argv,"controller_node");
    ros::NodeHandle nh;
    Controller controller = Controller(&nh);

    ros::spin();
}