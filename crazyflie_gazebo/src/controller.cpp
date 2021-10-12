#include <iostream>

#include <Eigen/Dense>
#include "controller.h"

#include <ros/ros.h>











void Controller::RLData_Callback(const crazyflie_msgs::RLData::ConstPtr &msg){
    _k_ep = msg->k_ep;
    if (msg->reset_flag == true){

        // RESET LOGGED FLIP VALUES
        _pos_flip << 0.0,0.0,0.0;
        _vel_flip << 0.0,0.0,0.0;
        _quat_flip << 1.0,0.0,0.0,0.0; // [qw,qx,qy,qz]
        _omega_flip << 0.0,0.0,0.0;

        _OF_y_flip = 0.0;
        _OF_x_flip = 0.0;
        _RREV_flip = 0.0;

        _flip_flag = false;
        _impact_flag = false;
        _policy_armed_flag = false;

    }
}


void Controller::RLCmd_Callback(const crazyflie_msgs::RLCmd::ConstPtr &msg){

    // CREATE CMD VECTOR AND VALS FROM SUBSCRIBED MESSAGE
    int cmd_type = msg->cmd_type;                       // Read cmd type from incoming message
    const geometry_msgs::Point vals = msg->cmd_vals;    // Construct vector from cmd values
    Vector3d cmd_vals(vals.x,vals.y,vals.z);
    float cmd_flag = msg->cmd_flag;                     // Construct flag from cmd flag value

    _ctrl_cmd << cmd_type,cmd_vals,cmd_flag; // Define cmd vector

    
    float eulx = 0.0;
    float euly = 0.0;
    float eulz = 0.0;
    

    switch(cmd_type){
        case 0: // Reset to home
        {
            _x_d << 0,0,0.4;
            _v_d << 0,0,0;
            _a_d << 0,0,0;

            _b1_d << 1,0,0;
            _omega_d << 0,0,0;

            _kp_xf = 1;
            _kd_xf = 1;
            _kp_Rf = 1;
            _kd_Rf = 1;


            
            _motorstop_flag = false;
            _Moment_flag = false;
            _policy_armed_flag = false;
            _flip_flag = false;
            _impact_flag = false;
            _eul_flag = false;

            _tumbled = false;
            // _tumble_detection = true;

            _slowdown_type = 0;
            Controller::adjustSimSpeed(_SIM_SPEED);


            break;
        }

        case 1: // Position
        {
            _x_d = cmd_vals;
            _kp_xf = cmd_flag;
            break;
        }

        case 2: // Velocity
        {
            _v_d = cmd_vals;
            _kd_xf = cmd_flag;
            break;
        }

        case 3: // Attitude 
        {    
            _eul_flag = cmd_flag;

            eulx = cmd_vals(0)*M_PI/180.0;
            euly = cmd_vals(1)*M_PI/180.0;
            eulz = cmd_vals(2)*M_PI/180.0;

            _R_d_custom = AngleAxisf(euly, Vector3f::UnitY()) * AngleAxisf(eulz, Vector3f::UnitZ()) * AngleAxisf(eulx, Vector3f::UnitX());
            break;
        }

        case 4: // Tumble-Detection
        {
            _tumble_detection = (bool)cmd_flag;
            break;
        }

        case 5: // Hard Set All Motorspeeds to Zero
        {
            _motorstop_flag = (bool)cmd_flag;
            break;
        }

        case 6: // Edit Gains 
        {
            if (cmd_vals(0) == 1)
            {
                P_kp_z = cmd_vals(1);
                P_kd_z = cmd_vals(2);
                P_ki_z = cmd_flag;
            }

            else if (cmd_vals(0) == 2)
            {
                P_kp_xy = cmd_vals(1);
                P_kd_xy = cmd_vals(2);
                P_ki_xy = cmd_flag;
            }

            else if (cmd_vals(0) == 3)
            {
                R_kp_xy = cmd_vals(1);
                R_kd_xy = cmd_vals(2);
                R_ki_xy = cmd_flag;
            }

            else if (cmd_vals(0) == 4)
            {
                R_kp_z = cmd_vals(1);
                R_kd_z = cmd_vals(2);
                R_ki_z = cmd_flag;
            }

            break;
        }

        case 7: // Execute Moment-Based Flip
        {
            _M_d = cmd_vals; // Convert from N*mm to N*m for calcs
            _Moment_flag = (bool)cmd_flag;
            break;
        }
        case 8: // Perform Policy Maneuver
        {
            _RREV_thr = cmd_vals(0);
            _G1 = cmd_vals(1);
            _G2 = cmd_vals(2);

            _policy_armed_flag = (bool)cmd_flag;
            break;


        }
        case 11: // Enable/Disable Stickyfoot
        {
            float sticky_cmd[4] = {-(float)cmd_type,cmd_flag,0,0};
            // This stickyfoot socket communication piggy-backs off of the motorspeed  
            // message & activates plugin when first number is negative but defines value
            // based off of the second number
            sendto(Ctrl_Mavlink_socket, sticky_cmd, sizeof(sticky_cmd),0, (struct sockaddr*)&addr_Mavlink, addr_Mavlink_len);
            break;
        }

        case 12:
        {
            _TEST_FLAG = true;
        }
    }

}


void Controller::controlThread()
{
    // =========== Controller Explanation =========== //
    // https://www.youtube.com/watch?v=A27knigjGS4&list=PL_onPhFCkVQhuPiUxUW2lFHB39QsavEEA&index=46
    // Derived from DOI's: 10.1002/asjc.567 (T. Lee) & 10.13140/RG.2.2.12876.92803/1 (M. Fernando)
   
    float motorspeed[4];
    


    // LOCAL STATE DECLARATIONS //
    Vector3d statePos;      // Current position [m]
    Vector3d stateVel;      // Current velocity [m/s]
    Vector4d stateQuat;     // Current attitude [rad] (quat form)
    Vector3d stateOmega;    // Current angular velocity [rad/s]

    Vector3d stateEul;      // Current attitude [rad] (roll, pitch, yaw angles)

    // UNUSED
    Vector3d stateVel_prev(0,0,0);  // Previous velocity [m/s]
    Vector3d stateAccel;            // Current acceleration [m/s^2]

    // LOCAL STATE PRESCRIPTIONS AND ERRORS //
    Vector3d x_d; // Pos-desired [m] 
    Vector3d v_d; // Velocity-desired [m/s]
    Vector3d a_d; // Acceleration-desired [m/s]
    

    Matrix3d R_d;               // Rotation-desired 
    Vector3d omega_d;           // Omega-desired [rad/s]
    Vector3d domega_d(0,0,0);   // Omega-Accl. [rad/s^2]

    Vector3d b1_d;  // Desired yaw direction of body-fixed axis (COM to prop #1)
    Vector3d b2_d;  // Desired body-fixed axis normal to b1 and b3
    Vector3d b3_d;  // Desired body-fixed vertical axis

    Vector3d e_x;   // Pos-Error [m]
    Vector3d e_v;   // Vel-error [m/s]
    Vector3d e_PI;  // Pos. Integral-error [m*s]


    Vector3d e_R;   // Rotation-error [rad]
    Vector3d e_w;   // Omega-error [rad/s]
    Vector3d e_RI;  // Rot. Integral-error [rad*s]


    

    Vector3d P_effort;          // Controller effort from positional gains
    Vector3d R_effort;          // Controller effort from rotational gains
    Vector3d F_thrust_ideal;    // Ideal thrust vector to minimize error   
    double F_thrust;
    Vector3d Gyro_dyn;          // Gyroscopic dynamics of system [Nm]
    Vector3d M;                 // Moment control vector [Nm]


    Vector4d FM;    // Thrust-Moment control vector (4x1) [N/N*mm]
    Vector4d f;     // Propeller thrust vector [g]


    
    Vector4d M_pwm;
    Vector4d motorspeed_Vec_d;  // Desired motorspeeds [rad/s]
    Vector4d motorspeed_Vec;    // Motorspeeds [rad/s]


    Vector3d b3; // Body-fixed vertical axis
    Quaterniond q;
    Matrix3d R; // Body-Global Rotation Matrix
    Vector3d e3(0,0,1); // Global z-axis

    

    float RREV = 0.0; // Relative Retinal Expansion Velocity [rad/s]
    float OF_x = 0.0; // Optical Flow about x-axis [rad/s]
    float OF_y = 0.0; // Optical Flow about y-axis [rad/s]

    // float RREV_flip = 0.0;
    // float OF_x_flip = 0.0;
    // float OF_y_flip = 0.0;
    ros::Time t_flip;


    float f_thrust_g = 0.0;
    float f_roll_g = 0.0;
    float f_pitch_g = 0.0;
    float f_yaw_g = 0.0;

    

    double M1_pwm = 0.0;
    double M2_pwm = 0.0;
    double M3_pwm = 0.0;
    double M4_pwm = 0.0;

    double MS1 = 0.0;
    double MS2 = 0.0;
    double MS3 = 0.0;
    double MS4 = 0.0;


    double yaw; // Z-axis [rad/s]
    double roll; // X-axis [rad/s]
    double pitch; // Y-axis [rad/s]

    
    double t = 0; // Time from Gazebo [s]
    unsigned int t_step = 0; // t_step counter



    // SYSTEM CONSTANTS
    float dt = (1.0/500.0);
    double m = _CF_MASS;   // Mass [kg] 
    double g = 9.81;  // Gravitational acceleration [m/s^2]

    double d = 0.047; // Distance from COM to prop [m]
    double dp = d*sin(M_PI/4);

    double kf = 2.21e-8; // Thrust constant [N/(rad/s)^2]
    double c_tf = 0.00612; // Moment Constant [Nm/N]

    Matrix3d J; // Rotational Inertia of CF
    // J<< 16.57,0.83,0.72,
    //     0.83,16.66,1.8,
    //     0.72,1.8,29.26;

    // J = J*1e-6;

    J<< 16.57,0.0,0.0,
        0.0,16.57,0.0,
        0.0,0.0,29.26;

    J = J*1e-6;

    

    float f_total = 0.0;
    // =========== ROS Definitions =========== //
    crazyflie_msgs::CtrlData ctrl_msg;
    ros::Rate rate(500);

    while(_isRunning)
    {

        // CONTROL GAINS
        Kp_P << P_kp_xy,P_kp_xy,P_kp_z;
        Kd_P << P_kd_xy,P_kd_xy,P_kd_z;
        Ki_P << P_ki_xy,P_ki_xy,P_ki_z;

        Kp_R << R_kp_xy,R_kp_xy,R_kp_z;
        Kd_R << R_kd_xy,R_kd_xy,R_kd_z;
        Ki_R << R_ki_xy,R_ki_xy,R_ki_z;


        // =========== STATE DEFINITIONS =========== //
        //  Define local state vectors from current class state vectors 
        t = _t;   
        statePos = _pos; 
        stateVel = _vel;
        stateOmega = _omega;
        stateQuat = _quat;

        RREV = _RREV;
        OF_y = _OF_y;
        OF_x = _OF_x;

        

        // =========== STATE SETPOINTS =========== //
        // Define control_cmd from recieved control_cmd
        x_d = _x_d;
        v_d = _v_d;
        a_d = _a_d;


        omega_d = _omega_d;         // Omega desired [rad/s]
        domega_d << 0.0,0.0,0.0;    // Omega-Accl. [rad/s^2]

        b1_d = _b1_d; // Desired body x-axis relative to global coords
        
        

        


        // =========== ROTATION MATRIX =========== //
        // R changes Body axes to be in terms of Global axes
        // https://www.andre-gaschler.com/rotationconverter/
        q.w() = stateQuat(0);
        q.vec() = stateQuat.segment(1,3);
        R = q.normalized().toRotationMatrix(); // Quaternion to Rotation Matrix Conversion
        b3 = R*e3; // Body vertical axis in terms of global axes
        
        // TUMBLE DETECTION
        if(b3(2) <= 0 && _tumble_detection == true){ // If e3 component of b3 is neg, turn motors off 
            _tumbled = true;
        }


        // =========== TRANSLATIONAL EFFORT =========== //
        e_x = statePos - x_d; 
        e_v = stateVel - v_d;

        // POS. INTEGRAL ERROR
        e_PI(0) += e_x(0)*dt;
        e_PI(0) = clamp(e_PI(0),-i_range_xy,i_range_xy);

        e_PI(1) += e_x(1)*dt;
        e_PI(1) = clamp(e_PI(1),-i_range_xy,i_range_xy);

        e_PI(2) += e_x(2)*dt;
        e_PI(2) = clamp(e_PI(2),-i_range_z,i_range_z);

        // Controller effort from errors and gain values
        P_effort = -Kp_P.cwiseProduct(e_x)*_kp_xf + -Kd_P.cwiseProduct(e_v)*_kd_xf + -Ki_P.cwiseProduct(e_PI)*_ki_xf; 
        F_thrust_ideal = P_effort + m*g*e3 + m*a_d; // Add feed-forward terms to get ideal control thrust vector



        // =========== DESIRED BODY AXES =========== // 
        b3_d = F_thrust_ideal.normalized();     // Desired body-fixed vertical axis
        b2_d = b3_d.cross(b1_d).normalized();   // Body-fixed horizontal axis

        R_d << b2_d.cross(b3_d).normalized(),b2_d,b3_d; // Desired rotational axis
                                                        // b2_d x b3_d != b1_d (look at derivation)
        
        if (_eul_flag == true){
            R_d = _R_d_custom.cast <double> ();
        }


        // =========== ROTATIONAL ERRORS =========== // 
        e_R = 0.5*dehat(R_d.transpose()*R - R.transpose()*R_d); // Rotational error

        e_w = stateOmega - R.transpose()*R_d*omega_d; // Ang vel error 
        // (Omega vecs are on different "space manifolds" so they need to be compared this way) - This is beyond me lol


        // ROT. INTEGRAL ERROR
        e_RI(0) += e_R(0)*dt;
        e_RI(0) = clamp(e_RI(0),-i_range_R_xy,i_range_R_xy);

        e_RI(1) += e_R(1)*dt;
        e_RI(1) = clamp(e_RI(1),-i_range_R_xy,i_range_R_xy);

        e_RI(2) += e_R(2)*dt;
        e_RI(2) = clamp(e_RI(2),-i_range_R_z,i_range_R_z);



        // =========== CONTROL EQUATIONS =========== // 
        F_thrust = F_thrust_ideal.dot(b3); // Thrust control value

        /* Gyro_dyn = [omega x (J*omega)] - [J*( hat(omega)*R'*R_d*omega_d - R'*R_d*domega_d )] */
        /* [M = -kp_R*e_R - kd_R*e_w -ki_R*e_RI + Gyro_dyn] */
        Gyro_dyn = stateOmega.cross(J*stateOmega) - J*(hat(stateOmega)*R.transpose()*R_d*omega_d - R.transpose()*R_d*domega_d); // Gyroscopic dynamics
        R_effort = -Kp_R.cwiseProduct(e_R)*_kp_Rf + -Kd_R.cwiseProduct(e_w)*_kd_Rf + -Ki_R.cwiseProduct(e_RI)*_ki_Rf;
        M = R_effort + Gyro_dyn; // Moment control vector


        if (!_tumbled){ // If not tumbled and tumble detection turned on
            
            if(_policy_armed_flag == true){
        
                if(RREV >= _RREV_thr && _flip_flag == false){

                    _RREV_flip = RREV;
                    _OF_x_flip = OF_x;
                    _OF_y_flip = OF_y;

                    t_flip = ros::Time::now();;
                    _pos_flip = statePos;
                    _vel_flip = stateVel;
                    _quat_flip = stateQuat;
                    _omega_flip = stateOmega;


                    _flip_flag = true;

                    
                }

                if(_flip_flag == true){

                    _M_d(0) = 0.0;
                    _M_d(1) = -_G1*1e-3;
                    _M_d(2) = 0.0;

                    M = _M_d*2; // Need to double moment to ensure it survives the PWM<0 cutoff
                    F_thrust = 0;

                    // RECORD MOTOR THRUST TYPES AT FLIP
                    _f_thrust_g_flip = F_thrust/4.0f*Newton2g;
                    _f_roll_g_flip = M[0]/(4.0f*dp)*Newton2g;
                    _f_pitch_g_flip = M[1]/(4.0f*dp)*Newton2g;
                    _f_yaw_g_flip = M[2]/(4.0f*c_tf)*Newton2g;
                }
            }
            else if(_Moment_flag == true){
                F_thrust = 0;
                M = _M_d;
            }
            else{
                F_thrust = F_thrust;
                M = M;
            }

        }
        else if(_tumbled == true && _tumble_detection == true){
            F_thrust = 0;
            M << 0.0,0.0,0.0;
        }

        FM << F_thrust,M*1e3; // Thrust-Moment control vector
    
        // =========== CONVERT THRUSTS AND MOMENTS TO MOTOR PWM VALUES =========== //                 
        f_thrust_g = F_thrust/4.0f*Newton2g;
        f_roll_g = M[0]/(4.0f*dp)*Newton2g;
        f_pitch_g = M[1]/(4.0f*dp)*Newton2g;
        f_yaw_g = M[2]/(4.0f*c_tf)*Newton2g;


        

        f_thrust_g = clamp(f_thrust_g,0.0,f_MAX*0.85);    // Clamp thrust to prevent control saturation

        // REPRESENT THRUST TYPES AS PERCENTAGE OF MAX THRUST
        f << f_thrust_g,f_roll_g,f_pitch_g,f_yaw_g;
        // f = f/f_MAX;
        f_total = (abs(f_thrust_g)+abs(f_roll_g)+abs(f_pitch_g)+abs(f_yaw_g))/f_MAX;

        // THRUST CONVERSION
        M1_pwm = limitPWM(thrust2PWM(f_thrust_g + f_roll_g - f_pitch_g + f_yaw_g));
        M2_pwm = limitPWM(thrust2PWM(f_thrust_g + f_roll_g + f_pitch_g - f_yaw_g));
        M3_pwm = limitPWM(thrust2PWM(f_thrust_g - f_roll_g + f_pitch_g + f_yaw_g));
        M4_pwm = limitPWM(thrust2PWM(f_thrust_g - f_roll_g - f_pitch_g - f_yaw_g));
        M_pwm << M1_pwm,M2_pwm,M3_pwm,M4_pwm;
        

        // CONVERT PWM VALUES TO MOTORSPEEDS 
        // The extra step here is just for data consistency between experiment and simulation
        
        // EXPERIMENT:
        // Desired Motor Thrusts => PWM values => Resulting Motor Thrusts

        // SIMULATION:
        // Desired Motor Thrusts => Motorspeeds => Motor Model Plugin => Resulting Motor Thrusts

        MS1 = sqrt(PWM2thrust(M1_pwm)*g2Newton/kf);
        MS2 = sqrt(PWM2thrust(M2_pwm)*g2Newton/kf);
        MS3 = sqrt(PWM2thrust(M3_pwm)*g2Newton/kf);
        MS4 = sqrt(PWM2thrust(M4_pwm)*g2Newton/kf);
        // motorspeed_Vec << 2127.178,2127.178,2127.178,2127.178;
        motorspeed_Vec << MS1,MS2,MS3,MS4;

        



        if(_motorstop_flag == true){ // Shutoff all motors
            motorspeed_Vec << 0,0,0,0;
        }


        // SIMULATION SLOWDOWN
        if(_LANDING_SLOWDOWN_FLAG==true && _k_ep>=_K_EP_SLOWDOWN){

            // WHEN CLOSE TO THE CEILING REDUCE SIM SPEED
            if(_H_CEILING-statePos(2)<=0.2 && _slowdown_type == 0){
                
                Controller::adjustSimSpeed(_SIM_SLOWDOWN_SPEED);
                _slowdown_type = 1;

            }

            // IF IMPACTED OR MISSED CEILING, INCREASE SIM SPEED TO DEFAULT
            if(_impact_flag == true && _slowdown_type == 1)
            {
                
                Controller::adjustSimSpeed(_SIM_SPEED);
                _slowdown_type = 2;
            }
            else if(stateVel(2) <= -0.5 && _slowdown_type == 1){
                Controller::adjustSimSpeed(_SIM_SPEED);
                _slowdown_type = 2;
            }

        }
        

    
        // DATA HANDLING
        if (t_step%_CTRL_OUTPUT_SLOWDOWN == 0){ // General Debugging output
        cout << setprecision(4) <<
        "t: " << _t << "\tCmd: " << _ctrl_cmd.transpose() << endl << 
        endl <<
        "RREV_thr: " << _RREV_thr << "\tG1: " << _G1 << "\tG2: " << _G2 << endl << 
        "RREV: " << RREV << "\tOF_x: " << OF_x << "\tOF_y: " << OF_y << endl <<
        "RREV_flip: " << _RREV_flip << "\tOF_x_tr: " << _OF_x_flip << "\tOF_y_tr: " << _OF_y_flip << endl << 
        endl << 
        "Kp_P: " << Kp_P.transpose() << "\tKp_R: " << Kp_R.transpose() << endl <<
        "Kd_P: " << Kd_P.transpose() << "\tKd_R: " << Kd_R.transpose() << endl <<
        "Ki_P: " << Ki_P.transpose() << "\tKi_R: " << Ki_R.transpose() << endl <<
        endl << 
        "i_range_xy: " << i_range_xy << "\ti_range_R_xy: " << i_range_R_xy << endl <<
        "i_range_z: " << i_range_z << "\ti_range_R_z: " << i_range_R_z << endl <<
        endl << 
        setprecision(0) <<
        "Policy_armed: " << _policy_armed_flag <<  "\tFlip_flag: " << _flip_flag << "\tImpact_flag: " << _impact_flag << endl <<
        "Tumble Detection: " << _tumble_detection << "\t\tTumbled: " << _tumbled << endl <<
        "kp_xf: " << _kp_xf << " \tkd_xf: " << _kd_xf << "\tkp_Rf: " << _kp_Rf << "\tkd_Rf: " << _kd_Rf  << endl <<
        "Slowdown_type: " << _slowdown_type << endl << 
        endl << setprecision(3) <<

        "x_d: " << x_d.transpose() << endl <<
        "v_d: " << v_d.transpose() << endl <<
        "omega_d: " << omega_d.transpose() << endl <<
        endl << 

        "Pos [m]: " << statePos.transpose() << "\te_x: " << e_x.transpose() << endl <<
        "Vel [m/s]: " << stateVel.transpose() << "\te_v: " << e_v.transpose() << endl <<
        "Omega [rad/s]: " << stateOmega.transpose() << "\te_w: " << e_w.transpose() << endl <<
        "e_PI: " << e_PI.transpose() << "\te_RI: " << e_RI.transpose() << endl <<
        endl << 

        "R:\n" << R << "\n\n" << 
        "R_d:\n" << R_d << "\n\n" << 
        // "Yaw: " << yaw*180/M_PI << "\tRoll: " << roll*180/M_PI << "\tPitch: " << pitch*180/M_PI << endl << // These values are wrong
        "e_R: " << e_R.transpose() << "\te_R (deg): " << e_R.transpose()*180/M_PI << endl <<
        endl <<

        "FM_d [N/N*mm]: " << FM.transpose() << endl << 
        "f [%]: " << f.transpose() << endl <<
        "f_total [%]: " << f_total << endl << 
        endl << setprecision(0) <<
        // "MS_d: " << motorspeed_Vec_d.transpose() << endl <<
        "MS: " << motorspeed_Vec.transpose() << endl <<
        "MS_PWM: " << M_pwm.transpose() << endl <<
        "=============== " << endl; 
        // printf("\033c"); // clears console window
        }

        Map<RowVector4f>(&motorspeed[0],1,4) = motorspeed_Vec.cast <float> (); // Converts motorspeeds to C++ array for data transmission
        if(_TEST_FLAG == false)
        {
        int len = sendto(Ctrl_Mavlink_socket, motorspeed, sizeof(motorspeed),0, // Send motorspeeds to Gazebo -> gazebo_motor_model
                (struct sockaddr*)&addr_Mavlink, addr_Mavlink_len); 
        }
        t_step++;
               

        ctrl_msg.RREV = RREV;
        ctrl_msg.OF_y = OF_y;
        ctrl_msg.OF_x = OF_x;


        
        // FLIP INFO
        ctrl_msg.flip_flag = _flip_flag;
        ctrl_msg.RREV_tr = _RREV_flip;
        ctrl_msg.OF_x_tr = _OF_x_flip;
        ctrl_msg.OF_y_tr = _OF_y_flip;
        ctrl_msg.FM_flip = {_f_thrust_g_flip,_f_roll_g_flip,_f_pitch_g_flip,_f_yaw_g_flip};


        ctrl_msg.Pose_tr.header.stamp = t_flip;             

        ctrl_msg.Pose_tr.pose.position.x = _pos_flip(0);
        ctrl_msg.Pose_tr.pose.position.y = _pos_flip(1);
        ctrl_msg.Pose_tr.pose.position.z = _pos_flip(2);

        ctrl_msg.Pose_tr.pose.orientation.x = _quat_flip(1);
        ctrl_msg.Pose_tr.pose.orientation.y = _quat_flip(2);
        ctrl_msg.Pose_tr.pose.orientation.z = _quat_flip(3);
        ctrl_msg.Pose_tr.pose.orientation.w = _quat_flip(0);

        ctrl_msg.Twist_tr.linear.x = _vel_flip(0);
        ctrl_msg.Twist_tr.linear.y = _vel_flip(1);
        ctrl_msg.Twist_tr.linear.z = _vel_flip(2);

        ctrl_msg.Twist_tr.angular.x = _omega_flip(0);
        ctrl_msg.Twist_tr.angular.y = _omega_flip(1);
        ctrl_msg.Twist_tr.angular.z = _omega_flip(2);

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
    controller.Load();
    ros::spin();
}