#include <iostream>

#include <Eigen/Dense>
#include "controller.h"

#include <ros/ros.h>



using namespace Eigen;
using namespace std;

void Controller::Load()
{
    cout << setprecision(3);
    cout << fixed;
    _isRunning = true;

    // INIT FIRST CONTROLLER SOCKET (COMMUNICATES W/ MAVLINK PORT:18080)
    Ctrl_Mavlink_socket = socket(AF_INET, SOCK_DGRAM, 0); // DGRAM is for UDP communication (Send data but don't care if it's recieved)
    Ctrl_Mavlink_socket_SNDBUF = 16;    // 4 floats [16 bytes] for Motorspeeds       
    Ctrl_Mavlink_socket_RCVBUF = 144;   // 18 doubles [144 bytes] for State array
    Ctrl_Mavlink_socket_PORT = 18070;   // Port for this socket

    // SET EXPECTED BUFFER SIZES
    if (setsockopt(Ctrl_Mavlink_socket, SOL_SOCKET, SO_SNDBUF, &Ctrl_Mavlink_socket_SNDBUF, sizeof(Ctrl_Mavlink_socket_SNDBUF))<0)
        cout<<"[FAILED] Ctrl_Mavlink_socket: Setting SNDBUF"<<endl;
    if (setsockopt(Ctrl_Mavlink_socket, SOL_SOCKET, SO_RCVBUF, &Ctrl_Mavlink_socket_RCVBUF, sizeof(Ctrl_Mavlink_socket_RCVBUF))<0)
        cout<<"[FAILED] Ctrl_Mavlink_socket: Setting RCVBUF"<<endl;
    int enable = 1; // Fix for error if socket hasn't close correctly when restarting program
    if (setsockopt(Ctrl_Mavlink_socket, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(int)) < 0)
        cout <<"help me"<< endl;

    // SET SOCKET SETTINGS
    memset(&addr_Ctrl_Mavlink, 0, sizeof(addr_Ctrl_Mavlink));
    addr_Ctrl_Mavlink.sin_family = AF_INET;
    addr_Ctrl_Mavlink.sin_addr.s_addr = htonl(INADDR_ANY);//inet_addr("0.0.0.0");
    addr_Ctrl_Mavlink.sin_port = htons(Ctrl_Mavlink_socket_PORT);

    // BIND ADDRESS TO CONTROLLER SOCKET (PORT:18070)
    if (bind(Ctrl_Mavlink_socket, (struct sockaddr*)&addr_Ctrl_Mavlink, sizeof(addr_Ctrl_Mavlink)) < 0)
        cout<<"[FAILED] Ctrl_Mavlink_socket: Binding address to socket"<<endl;
    else
        cout<<"[SUCCESS] Ctrl_Mavlink_socket: Binding address to socket"<<endl; 


    



    // INIT ADDRESS FOR MAVLINK SOCKET (PORT: 18080)
    Mavlink_PORT = 18080;
    memset(&addr_Mavlink, 0, sizeof(addr_Mavlink));
    addr_Mavlink.sin_family = AF_INET;
    addr_Mavlink.sin_addr.s_addr = htonl(INADDR_ANY);
    addr_Mavlink.sin_port = htons(Mavlink_PORT);
    addr_Mavlink_len = sizeof(addr_Mavlink);



    // MOTORSPEED TO MAVLINK TEST (VISUALLY CONFIRMS THINGS ARE WORKING IN SIM)
    float msg[4] = {1900,1900,1900,1900};
    int msg_len = 0;
    for(int k=0; k<2; k++)
        // To Gazebo socket, send msg of len(msg)
        msg_len = sendto(Ctrl_Mavlink_socket, msg, sizeof(msg),0, (struct sockaddr*)&addr_Mavlink, sizeof(addr_Mavlink));
    if(msg_len<0)
        cout<<"[FAILED] Ctrl_Mavlink_socket: Sending test motor speeds to Mavlink. Threads will mutual lock!"<<endl; // Not sure what mutual lock means
    else
        cout<<"[SUCCESS] Ctrl_Mavlink_socket: Sending test motor speeds to Mavlink. Avoiding mutual locking between threads!"<<endl;

    // START COMMUNICATION THREADS
    controllerThread = std::thread(&Controller::controlThread, this);


}


void Controller::global_stateCallback(const nav_msgs::Odometry::ConstPtr &msg){

    // SIMPLIFY STATE VALUES FROM TOPIC
    // Follow msg names from message details - "rqt -s rqt_msg" 
    
    const geometry_msgs::Point position = msg->pose.pose.position; 
    const geometry_msgs::Vector3 velocity = msg->twist.twist.linear;


    // SET STATE VALUES INTO CLASS STATE VARIABLES
    _t = msg->header.stamp.toSec();
    _pos << position.x, position.y, position.z;
    _vel << velocity.x, velocity.y, velocity.z;

    // This stuff should come from IMU callback but lockstep broke that topic for some reason
    const geometry_msgs::Quaternion quaternion = msg->pose.pose.orientation;
    const geometry_msgs::Vector3 omega = msg->twist.twist.angular;

    _quat << quaternion.w, quaternion.x, quaternion.y, quaternion.z, 
    _omega << omega.x, omega.y, omega.z;

}

void Controller::OFCallback(const nav_msgs::Odometry::ConstPtr &msg){

    const geometry_msgs::Point position = msg->pose.pose.position; 
    const geometry_msgs::Vector3 velocity = msg->twist.twist.linear;

    double h_ceiling = 2.5;
    double d = h_ceiling-position.z; // h_ceiling - height

    // SET SENSOR VALUES INTO CLASS VARIABLES
    // _RREV = msg->RREV;
    // _OF_x = msg->OF_x;
    // _OF_y = msg->OF_y;

    _RREV = velocity.z/d;
    _OF_x = -velocity.y/d;
    _OF_y = -velocity.x/d;
}

void Controller::imuCallback(const sensor_msgs::Imu::ConstPtr &msg){

    int a = 0;
}


void Controller::RLCmd_Callback(const crazyflie_rl::RLCmd::ConstPtr &msg){

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

            _x_d << 0,0,0.6;
            _v_d << 0,0,0;
            _a_d << 0,0,0;

            _b1_d << 1,0,0;
            _omega_d << 0,0,0;

            _kp_xf = 1;
            _kd_xf = 1;
            _kp_Rf = 1;
            _kd_Rf = 1;

            _kp_x = _kp_x_D;
            _kd_x = _kd_x_D;
            _kp_R = _kp_R_D;
            _kd_R = _kd_R_D;

            
            _motorstop_flag = false;
            _Moment_flag = false;
            _policy_armed_flag = false;
            _flip_flag = false;
            _eul_flag = false;

            _tumbled = false;
            // _tumble_detection = true;

            break;

        case 1: // Position
            _x_d = cmd_vals;
            _kp_xf = cmd_flag;
            break;

        case 2: // Velocity
            _v_d = cmd_vals;
            _kd_xf = cmd_flag;
            break;

        case 3: // Attitude 
            
            _eul_flag = cmd_flag;

            eulx = cmd_vals(0)*M_PI/180.0;
            euly = cmd_vals(1)*M_PI/180.0;
            eulz = cmd_vals(2)*M_PI/180.0;

            _R_d_custom = AngleAxisf(euly, Vector3f::UnitY()) * AngleAxisf(eulz, Vector3f::UnitZ()) * AngleAxisf(eulx, Vector3f::UnitX());
            break;
        
        case 4: // Tumble-Detection
            _tumble_detection = (bool)cmd_flag;
            break;

        case 5: // Hard Set All Motorspeeds to Zero
            _motorstop_flag = (bool)cmd_flag;
            break;

        case 6: // Edit Gains 

            _kp_x.fill(cmd_vals(0)); 
            _kd_x.fill(cmd_vals(1)); 
            _kp_R.fill(cmd_vals(2)); 
            _kd_R.fill(cmd_flag); 
            break;

        case 7: // Execute Moment-Based Flip

            _M_d = cmd_vals*1e-3; // Convert from N*mm to N*m for calcs
            _Moment_flag = (bool)cmd_flag;
            break;

        case 8: // Perform Policy Maneuver

            _RREV_thr = cmd_vals(0);
            _G1 = cmd_vals(1);
            _G2 = cmd_vals(2);

            _policy_armed_flag = (bool)cmd_flag;


        case 11: // Enable/Disable Stickyfoot
            float sticky_cmd[4] = {-(float)cmd_type,cmd_flag,0,0};
            // This stickyfoot socket communication piggy-backs off of the motorspeed  
            // message & activates plugin when first number is negative but defines value
            // based off of the second number
            sendto(Ctrl_Mavlink_socket, sticky_cmd, sizeof(sticky_cmd),0, (struct sockaddr*)&addr_Mavlink, addr_Mavlink_len);
    }

}

double sign(double x){
    if (x>0) return 1;
    if (x<0) return -1;
    return 0;
}



void Controller::controlThread()
{
    // =========== Controller Explanation =========== //
    // https://www.youtube.com/watch?v=A27knigjGS4&list=PL_onPhFCkVQhuPiUxUW2lFHB39QsavEEA&index=46
    // Derived from DOI's: 10.1002/asjc.567 (T. Lee) & 10.13140/RG.2.2.12876.92803/1 (M. Fernando)
   
    float motorspeed[4];
    


    // LOCAL STATE DECLARATIONS //
    Vector3d pos;   // Current position [m]
    Vector3d vel;   // Current velocity [m]
    Vector4d quat;  // Current attitude [rad] (quat form)
    Vector3d eul;   // Current attitude [rad] (roll, pitch, yaw angles)
    Vector3d omega; // Current angular velocity [rad/s]

    Vector3d pos_tr;    // Flip trigger position [m]
    Vector3d vel_tr;    // Flip trigger velocity [m]
    Vector4d quat_tr;   // Flip trigger attitude [rad] (quat form)
    Vector3d omega_tr;  // Flip trigger angular velocity [rad/s]
    

    // LOCAL STATE PRESCRIPTIONS AND ERRORS //
    Vector3d x_d; // Pos-desired [m] 
    Vector3d v_d; // Velocity-desired [m/s]
    Vector3d a_d; // Acceleration-desired [m/s]
    

    Matrix3d R_d; // Rotation-desired 
    Vector3d omega_d; // Omega-desired [rad/s]
    Vector3d domega_d(0,0,0); // Omega-Accl. [rad/s^2]

    Vector3d b1_d; // Desired yaw direction of body-fixed axis (COM to prop #1)
    Vector3d b2_d; // Desired body-fixed axis normal to b1 and b3
    Vector3d b3_d; // Desired body-fixed vertical axis

    Vector3d e_x; // Pos-Error [m]
    Vector3d e_v; // Vel-error  [m/s]
    Vector3d e_intg; // Integrated Pos-Error [m*s]
    Vector3d e_R; // Rotation-error [rad]
    Vector3d e_omega; // Omega-error [rad/s]


    

    
    Vector3d F_thrust_ideal; // Ideal thrust vector to minimize error   
    Vector3d Gyro_dyn; // Gyroscopic dynamics of system [Nm]
    Vector3d M; // Moment control vector [Nm]
    
    
    Vector4d FM; // Thrust-Moment control vector (4x1)
    Vector4d f; // Propeller thrusts [PWM]
    double F_thrust;


    
    Vector4d M_pwm;
    Vector4d motorspeed_Vec_d; // Desired motorspeeds [rad/s]
    Vector4d motorspeed_Vec; // Motorspeeds [rad/s]


    Vector3d b3; // Body-fixed vertical axis
    Quaterniond q;
    Matrix3d R; // Body-Global Rotation Matrix
    Vector3d e3(0,0,1); // Global z-axis
    


    
    // LOCAL CONTROLLER VARIABLES
    Vector3d kp_x;   // Pos. Gain
    Vector3d kd_x;   // Pos. derivative Gain
    Vector3d kp_R;   // Rot. Gain
    Vector3d kd_R;   // Rot. derivative Gain

    float RREV = 0.0;
    float OF_y = 0.0;
    float OF_x = 0.0;

    float OF_y_tr = 0.0;
    float OF_x_tr = 0.0;
    float RREV_tr = 0.0;
    double t_tr = 0.0;

    double Mx = 0.0;
    double My = 0.0;
    double Mz = 0.0;
    double F = 0.0;

    float f_thrust = 0.0;
    float f_roll = 0.0;
    float f_pitch = 0.0;
    float f_yaw = 0.0;

    int32_t f_thrust_pwm = 0;
    int32_t f_roll_pwm = 0;
    int32_t f_pitch_pwm = 0;
    int32_t f_yaw_pwm = 0;


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



    // might need to adjust weight to real case (sdf file too)
    double m = 0.0346;   // Mass [kg] 
    double g = 9.8066;  // Gravitational acceleration [m/s^2]
    double t = 0; // Time from Gazebo [s]
    unsigned int t_step = 0; // t_step counter



    // SYSTEM CONSTANTS
    double d = 0.040; // Distance from COM to prop [m]
    double dp = d*sin(M_PI/4);

    double kf = 2.2e-8; // Thrust constant [N/(rad/s)^2]
    double c_tf = 0.00612; // Moment Constant [Nm/N]

    Matrix3d J; // Rotational Inertia of CF
    J<< 1.65717e-05, 0, 0,
        0, 1.66556e-05, 0,
        0, 0, 2.92617e-05;


    // =========== ROS Definitions =========== //
    crazyflie_gazebo::CtrlData ctrl_msg;
    ros::Rate rate(500);

    while(_isRunning)
    {

        // =========== Control Definitions =========== //
        // Define control_cmd from recieved control_cmd
        x_d = _x_d;
        v_d = _v_d;
        a_d = _a_d;
        b1_d = _b1_d;
        omega_d = _omega_d;
        

        // =========== State Definitions =========== //
        //  Define local state vectors from current class state vectors 
        //      Note: This is just to be explicit with the program flow
        t = _t;   
        pos = _pos; 
        quat = _quat;
        vel = _vel;
        omega = _omega;

        kp_x = _kp_x;
        kd_x = _kd_x;
        kp_R = _kp_R;
        kd_R = _kd_R;

        RREV = _RREV;
        OF_y = _OF_y;
        OF_x = _OF_x;


        // =========== Rotation Matrix =========== //
        // R changes Body axes to be in terms of Global axes
        // https://www.andre-gaschler.com/rotationconverter/
        q.w() = quat(0);
        q.vec() = quat.segment(1,3);
        R = q.normalized().toRotationMatrix(); // Quaternion to Rotation Matrix Conversion
        
        yaw = atan2(R(1,0), R(0,0)); 
        roll = atan2(R(2,1), R(2,2)); 
        pitch = atan2(-R(2,0), sqrt(R(2,1)*R(2,1)+R(2,2)*R(2,2)));
    
        b3 = R*e3; // Body vertical axis in terms of global axes
        
        if(b3(2) <= 0 && _tumble_detection == true){ // If e3 component of b3 is neg, turn motors off 
            _tumbled = true;
        }


        // =========== Translational Errors & Desired Body-Fixed Axes =========== //
        e_x = pos - x_d; 
        e_v = vel - v_d;

        
        F_thrust_ideal = -kp_x.cwiseProduct(e_x)*_kp_xf + -kd_x.cwiseProduct(e_v)*_kd_xf + m*g*e3 + m*a_d; // ideal control thrust vector
        b3_d = F_thrust_ideal.normalized();     // Desired body-fixed vertical axis
        b2_d = b3_d.cross(b1_d).normalized();   // Body-fixed horizontal axis



        // =========== Rotational Errors =========== // 
        R_d << b2_d.cross(b3_d).normalized(),b2_d,b3_d; // Desired rotational axis
                                                        // b2_d x b3_d != b1_d (look at derivation)
        
        if (_eul_flag == true){
            
            R_d = _R_d_custom.cast <double> ();
        }
            

        e_R = 0.5*dehat(R_d.transpose()*R - R.transpose()*R_d); // Rotational error
        e_omega = omega - R.transpose()*R_d*omega_d; // Ang vel error 
        // (Omega vecs are on different "space manifolds" so they need to be compared this way) - This is beyond me lol
        


        // =========== Control Equations =========== // 
        F_thrust = F_thrust_ideal.dot(b3); // Thrust control value
        Gyro_dyn = omega.cross(J*omega) - J*(hat(omega)*R.transpose()*R_d*omega_d - R.transpose()*R_d*domega_d); // Gyroscopic dynamics
        M = -kp_R.cwiseProduct(e_R)*_kp_Rf + -kd_R.cwiseProduct(e_omega)*_kd_Rf + Gyro_dyn; // Moment control vector


        if (!_tumbled){ // If not tumbled and tumble detection turned on
            
            if(_policy_armed_flag == true){
        
                if(RREV >= _RREV_thr && _flip_flag == false){
                    OF_y_tr = OF_y;
                    OF_x_tr = OF_x;
                    RREV_tr = RREV;

                    t_tr = t;
                    pos_tr = pos;
                    vel_tr = vel;
                    quat_tr = quat;
                    omega_tr = omega;


                    _flip_flag = true;

                    // cout << "t: " << _t << endl;
                    // cout << "pos: " << _pos.transpose() << endl;
                    // cout << "vel: " << _vel.transpose() << endl;
                    // cout << "RREV: " << _RREV << endl;
                    // cout << endl;
                }

                if(_flip_flag == true){

                    _M_d(0) = 0.0;
                    _M_d(1) = -_G1*1e-3;
                    _M_d(2) = 0.0;

                    // Moment with base thrust
                    M = _M_d;

                    // Pure Moment
                    M = _M_d*2; // Need to double moment to ensure it survives the MS<0 cutoff
                    F_thrust = 0;
                }
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

        FM << F_thrust,M; // Thrust-Moment control vector
    
        // =========== Propellar Thrusts/Speeds =========== //
        f_thrust = F_thrust/4.0f;
        f_roll = M[0]/(4.0f*dp);
        f_pitch = M[1]/(4.0f*dp);
        f_yaw = M[2]/(4.0f*c_tf);

        f_thrust_pwm = thrust2PWM(f_thrust);
        f_roll_pwm = thrust2PWM(f_roll);
        f_pitch_pwm = thrust2PWM(f_pitch);
        f_yaw_pwm = thrust2PWM(f_yaw);

        f_thrust_pwm = clamp(f_thrust_pwm,0,(float)PWM_MAX*0.85f);

        f << f_thrust_pwm,f_roll_pwm,f_pitch_pwm,f_yaw_pwm;
        f = f/(float)PWM_MAX;

        M1_pwm = limitPWM(f_thrust_pwm + f_roll_pwm - f_pitch_pwm + f_yaw_pwm);
        M2_pwm = limitPWM(f_thrust_pwm + f_roll_pwm + f_pitch_pwm - f_yaw_pwm);
        M3_pwm = limitPWM(f_thrust_pwm - f_roll_pwm + f_pitch_pwm + f_yaw_pwm);
        M4_pwm = limitPWM(f_thrust_pwm - f_roll_pwm - f_pitch_pwm - f_yaw_pwm);

        // CONVERT PWM VALUES TO MOTORSPEEDS 
        // The extra step here is just for data consistency between experiment and simulation
        
        // EXPERIMENT:
        // Desired Motor Thrusts => PWM values => Resulting Motor Thrusts

        // SIMULATION:
        // Desired Motor Thrusts => Motorspeeds => Motor Model Plugin => Resulting Motor Thrusts

        M_pwm << M1_pwm,M2_pwm,M3_pwm,M4_pwm;


        MS1 = sqrt(PWM2thrust(M1_pwm)/kf);
        MS2 = sqrt(PWM2thrust(M2_pwm)/kf);
        MS3 = sqrt(PWM2thrust(M3_pwm)/kf);
        MS4 = sqrt(PWM2thrust(M4_pwm)/kf);


        motorspeed_Vec << MS1,MS2,MS3,MS4;

        if(_motorstop_flag == true){ // Shutoff all motors
            motorspeed_Vec << 0,0,0,0;
        }
        


        if (t_step%25 == 0){ // General Debugging output
        cout << setprecision(4) <<
        "t: " << _t << "\tCmd: " << _ctrl_cmd.transpose() << endl << 
        endl <<
        "RREV_thr: " << _RREV_thr << "\tG1: " << _G1 << "\tG2: " << _G2 << endl << 
        "RREV: " << RREV << "\tOF_x: " << OF_x << "\tOF_y: " << OF_y << endl <<
        "RREV_tr: " << RREV_tr << "\tOF_x_tr: " << 0.0 << "\tOF_y_tr: " << OF_y_tr << endl << 
        endl << 
        "kp_x: " << kp_x.transpose() << "\tkd_x: " << kd_x.transpose() << endl <<
        "kp_R: " << kp_R.transpose() << "\tkd_R: " << kd_R.transpose() << endl <<
        endl << 
        setprecision(1) <<
        "Policy_armed: " << _policy_armed_flag <<  "\t\tFlip_flag: " << _flip_flag << endl <<
        "Tumble Detection: " << _tumble_detection << "\t\tTumbled: " << _tumbled << endl <<
        "kp_xf: " << _kp_xf << " \tkd_xf: " << _kd_xf << "\tkp_Rf: " << _kp_Rf << "\tkd_Rf: " << _kd_Rf  << endl <<
        endl << setprecision(4) <<

        "x_d: " << x_d.transpose() << endl <<
        "v_d: " << v_d.transpose() << endl <<
        "omega_d: " << omega_d.transpose() << endl <<
        endl << 

        "pos: " << pos.transpose() << "\te_x: " << e_x.transpose() << endl <<
        "vel: " << vel.transpose() << "\te_v: " << e_v.transpose() << endl <<
        "omega: " << omega.transpose() << "\te_w: " << e_omega.transpose() << endl <<
        endl << 

        "R:\n" << R << "\n\n" << 
        "R_d:\n" << R_d << "\n\n" << 
        // "Yaw: " << yaw*180/M_PI << "\tRoll: " << roll*180/M_PI << "\tPitch: " << pitch*180/M_PI << endl << // These values are wrong
        "e_R: " << e_R.transpose() << "\te_R (deg): " << e_R.transpose()*180/M_PI << endl <<
        endl <<

        "FM_d: " << FM.transpose() << endl << 
        "f: " << f.transpose() << endl <<
        endl << setprecision(0) <<
        "MS_d: " << motorspeed_Vec_d.transpose() << endl <<
        "MS: " << motorspeed_Vec.transpose() << endl <<
        "MS_PWM: " << M_pwm.transpose() << endl <<
        "=============== " << endl; 
        printf("\033c"); // clears console window
        }

        Map<RowVector4f>(&motorspeed[0],1,4) = motorspeed_Vec.cast <float> (); // Converts motorspeeds to C++ array for data transmission
        int len = sendto(Ctrl_Mavlink_socket, motorspeed, sizeof(motorspeed),0, // Send motorspeeds to Gazebo -> gazebo_motor_model?
                (struct sockaddr*)&addr_Mavlink, addr_Mavlink_len); 
        
        t_step++;
        
        ctrl_msg.motorspeeds = {motorspeed[0],motorspeed[1],motorspeed[2],motorspeed[3]};
        ctrl_msg.flip_flag = _flip_flag;
        ctrl_msg.RREV_tr = RREV_tr;
        ctrl_msg.OF_y_tr = OF_y_tr;
        ctrl_msg.OF_x_tr = OF_x_tr;
        ctrl_msg.FM_flip = {FM[0],_M_d(0)*1e3,_M_d(1)*1e3,_M_d(2)*1e3};

        ctrl_msg.RREV = RREV;
        ctrl_msg.OF_y = OF_y;


        
        
        // This techinially converts to integer and micro-secs instead of nano-secs but I 
        // couldn't figure out the solution to do this the right way and keep it as nsecs
        ctrl_msg.Pose_tr.header.stamp.sec = int(t_tr);              // Integer portion of flip time as integer
        ctrl_msg.Pose_tr.header.stamp.nsec = int(t_tr*1000)%1000;   // Decimal portion of flip time as integer
        
        ctrl_msg.Pose_tr.pose.position.x = pos_tr(0);
        ctrl_msg.Pose_tr.pose.position.y = pos_tr(1);
        ctrl_msg.Pose_tr.pose.position.z = pos_tr(2);

        ctrl_msg.Pose_tr.pose.orientation.x = quat_tr(1);
        ctrl_msg.Pose_tr.pose.orientation.y = quat_tr(2);
        ctrl_msg.Pose_tr.pose.orientation.z = quat_tr(3);
        ctrl_msg.Pose_tr.pose.orientation.w = quat_tr(0);

        ctrl_msg.Twist_tr.linear.x = vel_tr(0);
        ctrl_msg.Twist_tr.linear.y = vel_tr(1);
        ctrl_msg.Twist_tr.linear.z = vel_tr(2);

        ctrl_msg.Twist_tr.angular.x = omega_tr(0);
        ctrl_msg.Twist_tr.angular.y = omega_tr(1);
        ctrl_msg.Twist_tr.angular.z = omega_tr(2);




        F = kf*(pow(motorspeed[1],2) + pow(motorspeed[2],2)
                + pow(motorspeed[0],2) + pow(motorspeed[3],2));
        Mx = kf*dp*(pow(motorspeed[0],2) + pow(motorspeed[1],2)
                    - pow(motorspeed[2],2) - pow(motorspeed[3],2))*1e3;
        My = kf*dp*(pow(motorspeed[1],2) + pow(motorspeed[2],2)
                    - pow(motorspeed[0],2) - pow(motorspeed[3],2))*1e3;
        Mz = kf*c_tf*(pow(motorspeed[1],2) - pow(motorspeed[2],2)
                    + pow(motorspeed[0],2) - pow(motorspeed[3],2))*1e3;
       
        

        ctrl_msg.FM = {F,Mx,My,Mz};

        // cout << F << " | "  << Mx << " | "  << My << " | "  << Mz << endl;
        

        
        
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