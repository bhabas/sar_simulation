#include <iostream>

#include <Eigen/Dense>
#include "controller.h"

#include <ros/ros.h>

void Controller::controllerGTCReset(void)
{
    // consolePrintf("GTC Reset\n");
    // Reset errors to zero
    e_PI = vzero();
    e_RI = vzero();

    x_d = mkvec(0.0f,0.0f,0.0f);
    v_d = mkvec(0.0f,0.0f,0.0f);
    a_d = mkvec(0.0f,0.0f,0.0f);
    

    tumbled = false;
    motorstop_flag = false;

    Moment_flag = false;
    // policy_armed_flag = false;
    flip_flag = false;

    // t = 0;
    // execute_traj = false;




}

void Controller::controllerGTC()
{

    

    // CONTROLLER GAIN VALUES
    // XY POSITION PID
    float P_kp_xy = 0.5f;
    float P_kd_xy = 0.4f;
    float P_ki_xy = 0.1f*0;
    float i_range_xy = 0.3f;

    // Z POSITION PID
    float P_kp_z = 1.2f;
    float P_kd_z = 0.35f;
    float P_ki_z = 0.1f*0;
    float i_range_z = 0.25f;

    // XY ATTITUDE PID
    float R_kp_xy = 0.004f;
    float R_kd_xy = 0.0017f;
    float R_ki_xy = 0.0f;
    float i_range_R_xy = 1.0f;

    // Z ATTITUDE PID
    float R_kp_z = 0.003f;
    float R_kd_z = 0.001f;
    float R_ki_z = 0.002*0;
    float i_range_R_z = 0.5f;

    // =========== ROS Definitions =========== //
    crazyflie_msgs::CtrlData ctrl_msg;
    ros::Rate rate(500);
       
    while(_isRunning)
    {
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
        statePos = mkvec(_position.x,_position.y,_position.z);                      // [m]
        stateVel = mkvec(_velocity.x,_velocity.y,_velocity.z);                      // [m]
        stateOmega = mkvec(_omega.x,_omega.y,_omega.z);   // [rad/s]
        stateQuat = mkquat(_quaternion.x,_quaternion.y,_quaternion.z,_quaternion.w);

        // =========== SENSOR DATA =========== // 
        RREV = _RREV;
        OF_x = _OF_x;
        OF_y = _OF_y;

        // EULER ANGLES EXPRESSED IN YZX NOTATION
        stateEul = quat2eul(stateQuat);
        stateEul.x = degrees(stateEul.x);
        stateEul.y = degrees(stateEul.y);
        stateEul.z = degrees(stateEul.z);


        // =========== STATE SETPOINTS =========== //
        x_d = mkvec(0.0f,0.0f,0.4f);    // Pos-desired [m]
        v_d = mkvec(0.0f,0.0f,0.0f);    // Vel-desired [m/s]
        a_d = mkvec(0.0f,0.0f,0.0f);    // Acc-desired [m/s^2]

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
        // cout << F_thrust << endl;
        M = vadd(R_effort,Gyro_dyn);            // Control moments [Nm]

        // =========== THRUST AND MOMENTS [FORCE NOTATION] =========== // 
        if(!tumbled){

            F_thrust = F_thrust;
            M = M;

        }
        else if(tumbled){
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

        
        M1_pwm = limitPWM(thrust2PWM(f_thrust_g + f_roll_g - f_pitch_g + f_yaw_g)); // Add respective thrust components and limit to (0 <= PWM <= 60,000)
        M2_pwm = limitPWM(thrust2PWM(f_thrust_g + f_roll_g + f_pitch_g - f_yaw_g));
        M3_pwm = limitPWM(thrust2PWM(f_thrust_g - f_roll_g + f_pitch_g + f_yaw_g));
        M4_pwm = limitPWM(thrust2PWM(f_thrust_g - f_roll_g - f_pitch_g - f_yaw_g));

        MS1 = sqrt(PWM2thrust(M1_pwm)*g2Newton/kf);
        MS2 = sqrt(PWM2thrust(M2_pwm)*g2Newton/kf);
        MS3 = sqrt(PWM2thrust(M3_pwm)*g2Newton/kf);
        MS4 = sqrt(PWM2thrust(M4_pwm)*g2Newton/kf);

        motorspeed[0] = MS1;
        motorspeed[1] = MS2;
        motorspeed[2] = MS3;
        motorspeed[3] = MS4;
        
        int len = sendto(Ctrl_Mavlink_socket, motorspeed, sizeof(motorspeed),0, // Send motorspeeds to Gazebo -> gazebo_motor_model
                (struct sockaddr*)&addr_Mavlink, addr_Mavlink_len); 
        

        ctrl_msg.RREV = RREV;
        ctrl_msg.OF_y = OF_y;
        ctrl_msg.OF_x = OF_x;

        ctrl_msg.FM = {f_thrust_g,f_roll_g,f_pitch_g,f_yaw_g};

        ctrl_Publisher.publish(ctrl_msg);
        rate.sleep();
    }
}


int main(int argc, char **argv)
{

    ros::init(argc, argv,"controller_node");
    ros::NodeHandle nh;
    Controller controller = Controller(&nh);
    controller.controllerGTCReset();
    controller.Load();
    ros::spin();
}