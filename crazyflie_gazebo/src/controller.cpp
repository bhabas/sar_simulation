
#include "controller.h"

// XY POSITION PID
float P_kp_xy = 0.05f*0.0f;
float P_kd_xy = 0.01f*0.0f;
float P_ki_xy = 0.0f;
float i_range_xy = 0.3f;

// Z POSITION PID
float P_kp_z = 1.2f;
float P_kd_z = 0.35f;
float P_ki_z = 0.0f;
float i_range_z = 0.25f;

// XY ATTITUDE PID
float R_kp_xy = 0.0015f;
float R_kd_xy = 0.0008f;
float R_ki_xy = 0.0f;
float i_range_R_xy = 1.0f;

// Z ATTITUDE PID
float R_kp_z = 0.003f;
float R_kd_z = 0.002f;
float R_ki_z = 0.000f;
float i_range_R_z = 0.5f;
void controllerGTCInit(void)
{
    controllerGTCTest();
    controllerGTCReset();
    printf("GTC Initiated\n");
}

void controllerGTCReset(void)
{
    printf("GTC Reset\n");
    // Reset errors to zero
    e_PI = vzero();
    e_RI = vzero();

    x_d = mkvec(0.0f,0.0f,0.0f);
    v_d = mkvec(0.0f,0.0f,0.0f);
    a_d = mkvec(0.0f,0.0f,0.0f);
    

    tumbled = false;
    motorstop_flag = false;

    Moment_flag = false;
    policy_armed_flag = false;
    flip_flag = false;

    t = 0;
    execute_traj = false;




}

bool controllerGTCTest(void)
{
    return true;
}

void GTC_Command(setpoint_t *setpoint)
{   

}

void controllerGTCTraj()
{

}



void controllerGTC(control_t *control, setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick,
                                         Controller* _CTRL) 
{
    if (RATE_DO_EXECUTE(RATE_500_HZ, tick)) {

        // if (setpoint->GTC_cmd_rec == true)
        //     {
                
        //         GTC_Command(setpoint);
        //         setpoint->GTC_cmd_rec = false;
        //     }

        // if (errorReset){
        //     controllerGTCReset();
        //     errorReset = false;
        //     }

        // if(execute_traj){
        //     controllerGTCTraj();
        // }

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

        // =========== STATE SETPOINTS =========== //
        x_d = mkvec(setpoint->position.x,setpoint->position.y,setpoint->position.z);             // Pos-desired [m]
        v_d = mkvec(setpoint->velocity.x, setpoint->velocity.y, setpoint->velocity.z);             // Vel-desired [m/s]
        a_d = mkvec(setpoint->acceleration.x, setpoint->acceleration.y, setpoint->acceleration.z); // Acc-desired [m/s^2]

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
        if (b3.z <= 0){
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
        temp2_v = veltmul(vneg(Kd_p), e_v);
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

        // printvec(M);

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

        MS1 = sqrt(PWM2thrust(M1_pwm)*g2Newton/kf);
        MS2 = sqrt(PWM2thrust(M2_pwm)*g2Newton/kf);
        MS3 = sqrt(PWM2thrust(M3_pwm)*g2Newton/kf);
        MS4 = sqrt(PWM2thrust(M4_pwm)*g2Newton/kf);

        

        

        _CTRL->MS_msg.MotorSpeeds = {(uint16_t)MS1,(uint16_t)MS2,(uint16_t)MS3,(uint16_t)MS4};
        _CTRL->MS_Publisher.publish(_CTRL->MS_msg);
        _CTRL->ctrl_Publisher.publish(_CTRL->ctrl_msg);

        // DATA HANDLING
        if (tick%5 == 0){ // General Debugging output
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
        "Policy_armed: " << policy_armed_flag <<  "\tFlip_flag: " << flip_flag << "\tImpact_flag: " << "impact_flag" << endl <<
        "Tumble Detection: " << "tumble_detection" << "\t\tTumbled: " << tumbled << endl <<
        "kp_xf: " << "kp_xf" << " \tkd_xf: " << "kd_xf" << endl <<
        "Slowdown_type: " << "slowdown_type" << endl << 
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
    // controller.controllerGTCReset();

    ros::spin();
}