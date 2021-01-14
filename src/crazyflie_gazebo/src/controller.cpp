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



void Controller::global_stateCallback(const gazebo_communication_pkg::GlobalState::ConstPtr &msg){

    // SIMPLIFY STATE VALUES FROM TOPIC
    // Follow msg names from message details - "rqt -s rqt_msg" 
    float _t = msg->header.stamp.toSec();
    const geometry_msgs::Point position = msg->global_pose.position; 
    const geometry_msgs::Quaternion quaternion = msg->global_pose.orientation;
    const geometry_msgs::Vector3 velocity = msg->global_twist.linear;
    const geometry_msgs::Vector3 omega = msg->global_twist.angular;

    // SET STATE VALUES INTO CLASS STATE VARIABLES
    _pos << position.x, position.y, position.z;
    _vel << velocity.x, velocity.y, velocity.z;
    _quat << quaternion.w, quaternion.x, quaternion.y, quaternion.z, 
    _omega << omega.x, omega.y, omega.z;


    // SET SENSOR VALUES INTO CLASS VARIABLES
    _RREV = msg->RREV;
    _OF_x = msg->OF_x;
    _OF_y = msg->OF_y;
    

}


void Controller::RLCmd_Callback(const crazyflie_rl::RLCmd::ConstPtr &msg){

    // CREATE CMD VECTOR AND VALS FROM SUBSCRIBED MESSAGE
    int cmd_type = msg->cmd_type;                       // Read cmd type from incoming message
    const geometry_msgs::Point vals = msg->cmd_vals;    // Construct vector from cmd values
    Vector3d cmd_vals(vals.x,vals.y,vals.z);
    float cmd_flag = msg->cmd_flag;                     // Construct flag from cmd flag value

    _ctrl_cmd << cmd_type,cmd_vals,cmd_flag; // Define cmd vector


    

    switch(cmd_type){
        case 0: // Reset to home

            _x_d << 0,0,0.3;
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
            break;

        case 1: // Position
            _x_d = cmd_vals;
            _kp_xf = cmd_flag;
            break;

        case 2: // Velocity
            _v_d = cmd_vals;
            _kd_xf = cmd_flag;
            break;

        case 3: // Attitude [Future implentation needed]
            break;
        
        case 4: // Execute Ang. Velocity-Based Flip [DEPRECATED]
                //      NOTE: This has been outperformed by moment based flips and removed
                //      Look back at old commits for reference if needed
            break;

        case 5: // Hard Set All Motorspeeds to Zero
            _motorstop_flag = (bool)cmd_flag;
            break;

        case 6: // Edit Gains [Needs Reimplemented]
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
    Vector3d pos; // Current position [m]
    Vector3d vel; // Current velocity [m]
    Vector4d quat; // Current attitude [rad] (quat form)
    Vector3d eul; // Current attitude [rad] (roll, pitch, yaw angles)
    Vector3d omega; // Current angular velocity [rad/s]
    

    // LOCAL STATE PRESCRIPTIONS AND ERRORS //
    Vector3d x_d; // Pos-desired [m] 
    Vector3d v_d; // Velocity-desired [m/s]
    Vector3d a_d; // Acceleration-desired [m/s]
    

    Matrix3d R_d; // Rotation-desired 
    Matrix3d R_d_custom; // Rotation-desired (ZXY Euler angles)
    Vector3d eul_d; // Desired attitude (ZXY Euler angles) [rad] (roll, pitch, yaw angles)
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
    Vector4d f; // Propeller thrusts [N]
    double F_thrust;


    

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

     


    double yaw; // Z-axis [rad/s]
    double roll; // X-axis [rad/s]
    double pitch; // Y-axis [rad/s]



    // might need to adjust weight to real case (sdf file too)
    double m = 0.026 + 0.00075*4; // Mass [kg]
    double g = 9.8066; // Gravitational acceleration [m/s^2]
    double t = 0; // Time from Gazebo [s]
    unsigned int t_step = 0; // t_step counter



    // SYSTEM CONSTANTS
    double d = 0.040; // Distance from COM to prop [m]
    double d_p = d*sin(M_PI/4);

    double kf = 2.2e-8; // Thrust constant [N/(rad/s)^2]
    double c_Tf = 0.00612; // Moment Constant [Nm/N]

    Matrix3d J; // Rotational Inertia of CF
    J<< 1.65717e-05, 0, 0,
        0, 1.66556e-05, 0,
        0, 0, 2.92617e-05;

    Matrix4d Gamma; // Thrust-Moment control vector conversion matrix
    Gamma << 1,     1,     1,     1, // Motor thrusts = Gamma*Force/Moment vec
             d_p,   d_p,  -d_p,  -d_p, 
            -d_p,   d_p,   d_p,  -d_p, 
            c_Tf,  -c_Tf, c_Tf,  -c_Tf;
    Matrix4d Gamma_I = Gamma.inverse(); // Calc here once to reduce calc load



    // =========== ROS Definitions =========== //
    crazyflie_gazebo::CtrlData ctrl_msg;
    ros::Rate rate(800);

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


        // =========== Translational Errors & Desired Body-Fixed Axes =========== //
        e_x = pos - x_d; 
        e_v = vel - v_d;

        
        F_thrust_ideal = -kp_x.cwiseProduct(e_x)*_kp_xf + -kd_x.cwiseProduct(e_v)*_kd_xf + m*g*e3 + m*a_d; // ideal control thrust vector
        b3_d = F_thrust_ideal.normalized();     // Desired body-fixed vertical axis
        b2_d = b3_d.cross(b1_d).normalized();   // Body-fixed horizontal axis



        // =========== Rotational Errors =========== // 
        R_d << b2_d.cross(b3_d).normalized(),b2_d,b3_d; // Desired rotational axis
                                                        // b2_d x b3_d != b1_d (look at derivation)

        e_R = 0.5*dehat(R_d.transpose()*R - R.transpose()*R_d); // Rotational error
        e_omega = omega - R.transpose()*R_d*omega_d; // Ang vel error 
        // (Omega vecs are on different "space manifolds" so they need to be compared this way) - This is beyond me lol
        


        // =========== Control Equations =========== // 
        F_thrust = F_thrust_ideal.dot(b3); // Thrust control value
        Gyro_dyn = omega.cross(J*omega) - J*(hat(omega)*R.transpose()*R_d*omega_d - R.transpose()*R_d*domega_d); // Gyroscopic dynamics
        M = -kp_R.cwiseProduct(e_R)*_kp_Rf + -kd_R.cwiseProduct(e_omega)*_kd_Rf + Gyro_dyn; // Moment control vector

        if(_Moment_flag == true){
            FM << F_thrust,_M_d;
        }
        else{

            if(_policy_armed_flag == true){
            
                if(_RREV >= _RREV_thr){
                    M(0) = 0.0;
                    M(1) = ( (_G1*1e-1)*_RREV - (_G2*1e-1)*abs(_OF_y))*sign(_OF_y)*1e-3;
                    M(2) = 0.0;

                    _flip_flag = true;
                }
            }

            FM << F_thrust,M; // Thrust-Moment control vector
        }

        
        

        
        

        // =========== Propellar Thrusts/Speeds =========== //
        f = Gamma_I*FM; // Propeller thrusts
        motorspeed_Vec_d = (1/kf*f).array().sqrt(); // Calculated motorspeeds
        motorspeed_Vec = motorspeed_Vec_d; // Actual motorspeeds to be capped


        // Cap motor thrusts between 0 and 2500 rad/s
        for(int k_motor=0;k_motor<4;k_motor++) 
        {
            if(motorspeed_Vec(k_motor)<0){
                motorspeed_Vec(k_motor) = 0;
            }
            else if(isnan(motorspeed_Vec(k_motor))){
                motorspeed_Vec(k_motor) = 0;
            }
            else if(motorspeed_Vec(k_motor)>= 2500){ // Max rotation speed (rad/s)
                // cout << "Motorspeed capped - Motor: " << k_motor << endl;
                motorspeed_Vec(k_motor) = 2500;
            }
        }

        if(b3(2) <= 0){ // If e3 component of b3 is neg, turn motors off 
            _motorstop_flag = true;
        }


        if(_motorstop_flag == true){ // Shutoff all motors
            motorspeed_Vec << 0,0,0,0;
        }
        


        if (t_step%75 == 0){ // General Debugging output
        cout << setprecision(4) <<
        "t: " << _t << "\tCmd: " << _ctrl_cmd.transpose() << endl << 
        endl <<
        "RREV: " << _RREV << "\tOF_x: " << _OF_x << "\tOF_y: " << _OF_y << endl <<
        "RREV_thr: " << _RREV_thr << "\tG1: " << _G1 << "\tG2: " << _G2 << endl << 
        endl << 
        "kp_x: " << _kp_x.transpose() << "\tkd_x: " << _kd_x.transpose() << endl <<
        "kp_R: " << _kp_R.transpose() << "\tkd_R: " << _kd_R.transpose() << endl <<
        endl << 
        setprecision(1) <<
        "Policy_armed: " << _policy_armed_flag <<  "\tFlip_flag:" << _flip_flag << endl <<
        "motorstop_flag: " << _motorstop_flag << "\tMoment_flag: " << _Moment_flag << endl <<
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
        "Yaw: " << yaw*180/M_PI << "\tRoll: " << roll*180/M_PI << "\tPitch: " << pitch*180/M_PI << endl <<
        "e_R: " << e_R.transpose() << "\te_R (deg): " << e_R.transpose()*180/M_PI << endl <<
        endl <<

        "FM: " << FM.transpose() << endl <<
        "f: " << f.transpose() << endl <<
        endl << setprecision(0) <<
        "MS_d: " << motorspeed_Vec_d.transpose() << endl <<
        "MS: " << motorspeed_Vec.transpose() << endl <<
        "=============== " << endl; 
        printf("\033c"); // clears console window
        }

        Map<RowVector4f>(&motorspeed[0],1,4) = motorspeed_Vec.cast <float> (); // Converts motorspeeds to C++ array for data transmission
        int len = sendto(Ctrl_Mavlink_socket, motorspeed, sizeof(motorspeed),0, // Send motorspeeds to Gazebo -> gazebo_motor_model?
                (struct sockaddr*)&addr_Mavlink, addr_Mavlink_len); 
        
        t_step++;
        
        ctrl_msg.motorspeeds = {motorspeed[0],motorspeed[1],motorspeed[2],motorspeed[3]};
        ctrl_msg.flip_flag = _flip_flag;
        ctrl_msg.FM_d = {FM[0],FM[1],FM[2],FM[3]}
        
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