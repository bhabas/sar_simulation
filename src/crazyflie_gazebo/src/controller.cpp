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


    


    // INIT SECOND CONTROLLER SOCKET (COMMUNICATES W/ RL PORT:18050)
    Ctrl_RL_socket = socket(AF_INET, SOCK_DGRAM, 0);
    Ctrl_RL_socket_SNDBUF = 144; // 18 doubles [144 bytes] for State array
    Ctrl_RL_socket_RCVBUF = 40;  // 5 doubles [8 bytes] for Controller Commands
    Ctrl_RL_socket_Port = 18060; // Port for this socket

    // SET EXPECTED BUFFER SIZES
    if (setsockopt(Ctrl_RL_socket, SOL_SOCKET, SO_SNDBUF, &Ctrl_RL_socket_SNDBUF, sizeof(Ctrl_RL_socket_SNDBUF))<0)
        cout<<"[FAILED] Ctrl_RL_socket: Setting SNDBUF"<<endl;
    if (setsockopt(Ctrl_RL_socket, SOL_SOCKET, SO_RCVBUF, &Ctrl_RL_socket_RCVBUF, sizeof(Ctrl_RL_socket_RCVBUF))<0)
        cout<<"[FAILED] Ctrl_RL_socket: Setting RCVBUF"<<endl;
    // Fix for error if socket hasn't close correctly when restarting program
    if (setsockopt(Ctrl_RL_socket, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(int)) < 0)
        cout <<"help me"<< endl;
    

    // SET SOCKET SETTINGS
    memset(&addr_Ctrl_RL, 0, sizeof(addr_Ctrl_RL)); // Not sure what this does
    addr_Ctrl_RL.sin_family = AF_INET; // IPv4 Format
    addr_Ctrl_RL.sin_addr.s_addr = htonl(INADDR_ANY);//inet_addr("127.0.0.1");
    addr_Ctrl_RL.sin_port = htons(Ctrl_RL_socket_Port); // RL Port number
    
    
    // BIND ADDRESS TO SECOND CONTROLLER SOCKET (PORT:18060)
    if (bind(Ctrl_RL_socket, (struct sockaddr*)&addr_Ctrl_RL, sizeof(addr_Ctrl_RL))<0)
        cout<<"[FAILED] Ctrl_RL_socket: Binding address to socket"<<endl;
    else
        cout<<"[SUCCESS] Ctrl_RL_socket: Binding address to socket"<<endl; 

    // INIT ADDRESS FOR MAVLINK SOCKET (PORT: 18080)
    Mavlink_PORT = 18080;
    memset(&addr_Mavlink, 0, sizeof(addr_Mavlink));
    addr_Mavlink.sin_family = AF_INET;
    addr_Mavlink.sin_addr.s_addr = htonl(INADDR_ANY);
    addr_Mavlink.sin_port = htons(Mavlink_PORT);
    addr_Mavlink_len = sizeof(addr_Mavlink);

    // INIT ADDRESS FOR RL SOCKET (PORT:18050)
    RL_PORT = 18050;
    memset(&addr_RL, 0, sizeof(addr_RL));
    addr_RL.sin_family = AF_INET;
    addr_RL.sin_addr.s_addr = htonl(INADDR_ANY);
    addr_RL.sin_port = htons(RL_PORT);
    addr_RL_len = sizeof(addr_RL);

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
    receiverThread_RL = std::thread(&Controller::recvThread_RL, this);
    controllerThread = std::thread(&Controller::controlThread, this);


}

void Controller::recvThread_RL()
{
    float motorspeed_fake[4] = {0,0,0,0};

    while(_isRunning)
    {
        //cout<<"[recvThread_RL] Receiving command from RL"<<endl;
        int len = recvfrom(Ctrl_RL_socket, control_cmd_recvd, sizeof(control_cmd_recvd),0, (struct sockaddr*)&addr_RL, &addr_RL_len);


        if(control_cmd_recvd[0]>10) // If header is 11 then enable sticky
        {
            motorspeed_fake[0] = -control_cmd_recvd[0];
            motorspeed_fake[1] = control_cmd_recvd[1];
            //cout<<"Send sticky command command: "<< motorspeed_fake[0]<<", "<<motorspeed_fake[1]<<endl;
            sendto(Ctrl_Mavlink_socket, motorspeed_fake, sizeof(motorspeed_fake),0, (struct sockaddr*)&addr_Mavlink, addr_Mavlink_len);

            
        }
    }
}

void Controller::callback_number(const gazebo_communication_pkg::GlobalState::ConstPtr &msg){

    // SIMPLIFY STATE VALUES FROM TOPIC
    float _t = msg->header.stamp.toSec();
    const geometry_msgs::Point position = msg->global_pose.position; // Follow names from message details - "rqt -s rqt_msg" 
    const geometry_msgs::Quaternion quaternion = msg->global_pose.orientation;
    const geometry_msgs::Vector3 velocity = msg->global_twist.linear;
    const geometry_msgs::Vector3 omega = msg->global_twist.angular;

    // SET STATE VALUES INTO CLASS STATE VARIABLES
    _pos << position.x, position.y, position.z;
    _vel << velocity.x, velocity.y, velocity.z;
    _quat << quaternion.w, quaternion.x, quaternion.y, quaternion.z, 
    _omega << omega.x, omega.y, omega.z;


    // std::cout << _vel.transpose() << std::endl;

}






void Controller::controlThread()
{
    // =========== Controller Explanation =========== //
    // https://www.youtube.com/watch?v=A27knigjGS4&list=PL_onPhFCkVQhuPiUxUW2lFHB39QsavEEA&index=46
    // Derived from DOI's: 10.1002/asjc.567 (T. Lee) & 10.13140/RG.2.2.12876.92803/1 (M. Fernando)
    typedef Matrix<double, 3, 3, RowMajor> RowMatrix3d; 

    
    MotorCommand motorspeed_structure;

    float motorspeed[4];
    double state_full[18];
    

    int type; // Command type {1:Pos, 2:Vel, 3:Att, 4:Omega, 5:Stop}
    double ctrl_flag; // On/Off switch for controller
    double control_cmd[5];
    Vector3d control_vals;
    

    // State Declarations
    Vector3d pos; // Current position [m]
    Vector3d vel; // Current velocity [m]
    Vector4d quat_Eig; // Current attitude [rad] (quat form)
    Vector3d eul; // Current attitude [rad] (roll, pitch, yaw angles)
    Vector3d omega; // Current angular velocity [rad/s]

    

    // Default desired States
    Vector3d x_d_Def(0,0,0.3); // Pos-desired (Default) [m]  # Should be z=0.03 but needs integral controller for error offset
    Vector3d v_d_Def(0,0,0); // Velocity-desired (Default) [m/s]
    Vector3d a_d_Def(0,0,0); // Acceleration-desired (Default) [m/s]
    Vector3d b1_d_Def(1,0,0); // Desired global pointing direction (Default)
    Vector3d omega_d_Def(0,0,0);
    

    // State Error and Prescriptions
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


    Vector3d e3(0,0,1); // Global z-axis

    
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
    


    
    // Controller Values
    Vector4d Ctrl_Gains; 
    Vector3d kp_x(0.1,0.1,0.11);         // Pos. Gain
    Vector3d kd_x(0.08,0.08,0.08);      // Pos. derivative Gain
    Vector3d kp_R(0.05,0.05,0.05);      // Rot. Gain
    Vector3d kd_R(0.005,0.005,0.005);   // Rot. derivative Gain

    Vector3d kp_omega(0.005,0.005 ,0); // Flip proportional Gain
    // Omega proportional gain (similar to kd_R but that's for damping and this is to achieve omega_d)
    // kd_R is great for stabilization but for flip manuevers it's too sensitive and 
    // saturates the motors causing instability during the rotation

    // Controller Flags
    double kp_xf = 1; // Pos. Gain Flag
    double kd_xf = 1; // Pos. derivative Gain Flag
    double kp_Rf = 1; // Rot. Gain Flag
    double kd_Rf = 1; // Rot. derivative Gain Flag

    


    double yaw; // Z-axis [rad/s]
    double roll; // X-axis [rad/s]
    double pitch; // Y-axis [rad/s]



    // might need to adjust weight to real case (sdf file too)
    double m = 0.026 + 0.00075*4; // Mass [kg]
    double g = 9.8066; // Gravitational acceleration [m/s^2]
    double t = 0; // Time from Gazebo [s]
    double t_prev = 0; // Prev time val [s]
    double dt = 0;  // Time difference [s]
    unsigned int t_step = 0; // t_step counter



    // System Constants
    double d = 0.040; // Distance from COM to prop [m]
    double d_p = d*sin(M_PI/4);

    double kf = 2.21e-8; // Thrust constant [N/(rad/s)^2]
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



  
    
    // 1 is on | 0 is off by default
    // double att_control_flag = 0; // Controls implementation
    double flip_flag = 1;       // Controls thrust implementation
    double motorstop_flag = 0;  // Controls stop implementation



   
    double w; // Angular frequency for trajectories [rad/s]
    double b; // Amplitude for trajectories [m]

    // =========== Trajectory Definitions =========== //
    x_d << x_d_Def;
    v_d << v_d_Def;
    a_d << a_d_Def;
    b1_d << b1_d_Def;

    crazyflie_gazebo::CtrlData ctrl_msg;
    ros::Rate rate(1000);

    while(_isRunning)
    {

        // =========== Control Definitions =========== //
        // Define control_cmd from recieved control_cmd
        if (control_cmd_recvd[0]!=11) // Change to != 10 to check if not a sticky foot command
            memcpy(control_cmd, control_cmd_recvd, sizeof(control_cmd)); // Compiler doesn't work without this line for some reason? 
            Map<Matrix<double,5,1>> control_cmd_Eig(control_cmd_recvd); 

        type = control_cmd_Eig(0); // Command type
        control_vals = control_cmd_Eig.segment(1,3); // Command values
        ctrl_flag = control_cmd_Eig(4); // Controller On/Off switch

        switch(type){ // Define Desired Values

            case 0: // Reset all changes to default vals and return to home pos
                x_d << x_d_Def;
                v_d << v_d_Def;
                a_d << a_d_Def;
                b1_d << b1_d_Def;
                omega_d << omega_d_Def;

          

                // kp_x = 0.1;   // Pos. Gain
                // kd_x = 0.1;  // Pos. derivative Gain
                // kp_R = 0.05;  // Rot. Gain // Keep checking rotational speed
                kd_R<< 0.005,0.005,0.005; // Rot. derivative Gain

                kp_xf=ctrl_flag; // Reset control flags
                kd_xf=ctrl_flag;
                kp_Rf=ctrl_flag;
                kd_Rf=ctrl_flag; 

                motorstop_flag=0;
                flip_flag=1;
                // att_control_flag=0;
                break;

            case 1: // Position
                x_d << control_vals;
                kp_xf = ctrl_flag;
                break;

            case 2: // Velocity
                v_d << control_vals;
                kd_xf = ctrl_flag;
                break;

            case 3: // Attitude [Implementation needs to be finished]
                eul_d << control_vals;
                kp_Rf = ctrl_flag;

                // att_control_flag = ctrl_flag;
                break;

            case 4: // Exectute Flip
                kd_R = kp_omega; // Change to flip gain

                omega_d << control_vals;
                kp_xf = 0; // Turn off other controllers
                kd_xf = 0;
                kp_Rf = 0;
                kd_Rf = ctrl_flag; // Turn control on and change error calc
                
                flip_flag = 0; // Turn thrust control off
                break;

            case 5: // Stop Motors
                motorstop_flag = ctrl_flag;
                break;

            case 6: // Reassign new control gains
                Ctrl_Gains << control_vals,ctrl_flag;
                // kp_x = Ctrl_Gains(0);
                // kd_x = Ctrl_Gains(1);
                // kp_R = Ctrl_Gains(2);
                // kd_R = Ctrl_Gains(3);
                break;
        }

        // =========== State Definitions =========== //


        //  Define local state vectors from current class state vectors 
        //      Note: This is just to be explicit with the program flow
        t = _t;   
        pos = _pos; 
        quat_Eig = _quat;
        vel = _vel;
        omega = _omega;


        // =========== Rotation Matrix =========== //
        // R changes Body axes to be in terms of Global axes
        // https://www.andre-gaschler.com/rotationconverter/
        q.w() = quat_Eig(0);
        q.vec() = quat_Eig.segment(1,3);
        R = q.normalized().toRotationMatrix(); // Quaternion to Rotation Matrix Conversion
        
        yaw = atan2(R(1,0), R(0,0)); 
        roll = atan2(R(2,1), R(2,2)); 
        pitch = atan2(-R(2,0), sqrt(R(2,1)*R(2,1)+R(2,2)*R(2,2)));
    
        b3 = R*e3; // body vertical axis in terms of global axes


        // =========== Translational Errors & Desired Body-Fixed Axes =========== //
        e_x = pos - x_d; 
        e_v = vel - v_d;

        
        F_thrust_ideal = -kp_x.cwiseProduct(e_x)*kp_xf + -kd_x.cwiseProduct(e_v)*kd_xf + m*g*e3 + m*a_d; // ideal control thrust vector
        b3_d = F_thrust_ideal.normalized(); // desired body-fixed vertical axis
        b2_d = b3_d.cross(b1_d).normalized(); // body-fixed horizontal axis



        // =========== Rotational Errors =========== // 
        R_d << b2_d.cross(b3_d).normalized(),b2_d,b3_d; // Desired rotational axis
                                                        // b2_d x b3_d != b1_d (look at derivation)

        // if (att_control_flag == 1){ // [Attitude control will be implemented here]
        //     R_d = R_d_custom;
        // }

        e_R = 0.5*dehat(R_d.transpose()*R - R.transpose()*R_d); // Rotational error
        e_omega = omega - R.transpose()*R_d*omega_d; // Ang vel error 
        // (Omega vecs are on different "space manifolds" so they need to be compared this way) - This is beyond me lol
        


        // =========== Control Equations =========== // 
        F_thrust = F_thrust_ideal.dot(b3)*(flip_flag); // Thrust control value
        Gyro_dyn = omega.cross(J*omega) - J*(hat(omega)*R.transpose()*R_d*omega_d - R.transpose()*R_d*domega_d); // Gyroscopic dynamics
        M = -kp_R.cwiseProduct(e_R)*kp_Rf + -kd_R.cwiseProduct(e_omega)*kd_Rf + Gyro_dyn; // Moment control vector
        FM << F_thrust,M; // Thrust-Moment control vector

        
        

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

        if(b3(2) <= 0){ // If e3 component of b3 is neg, turn motors off [arbitrary amount]
            motorstop_flag = 1;
        }


        if(motorstop_flag == 1){ // Shutoff all motors
            motorspeed_Vec << 0,0,0,0;
        }
        


        if (t_step%100 == 0){ // General Debugging output
        cout << setprecision(4) <<
        "t: " << t << "\tCmd: " << control_cmd_Eig.transpose() << endl << 
        endl <<
        "kp_x: " << kp_x.transpose() << "\tkd_x: " << kd_x.transpose() << endl <<
        "kp_R: " << kp_R.transpose() << "\tkd_R: " << kd_R.transpose() << endl <<
        "kp_omega (flip): " << kp_omega.transpose() << endl <<
        setprecision(1) <<
        "flip_flag: " << flip_flag << "\tmotorstop_flag: " << motorstop_flag << endl <<
        "kp_xf: " << kp_xf << " \tkd_xf: " << kd_xf << "\tkp_Rf: " << kp_Rf << "\tkd_Rf: " << kd_Rf  << endl <<
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
        t_prev = t;
        
        ctrl_msg.motorspeeds = {motorspeed[0],motorspeed[1],motorspeed[2],motorspeed[3]};
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