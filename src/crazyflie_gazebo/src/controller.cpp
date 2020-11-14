#include <iomanip>      // provides std::setprecision
#include <math.h>
#include <algorithm>
#include <stdint.h>
#include <Eigen/Dense>
#include "controller.h"


using namespace Eigen;
using namespace std;


void Controller::Load()
{   cout << setprecision(3);
    cout << fixed;
    isRunning = true;

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
        cout<<"[FAILED] Socket binding for Ctrl_RL failed!"<<endl;
    else
        cout<<"[SUCCESS] Socket binding for Ctrl_RL succeeded!"<<endl;




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
    float msg[4] = {100.0,100.0,100.0,100.0};
    int msg_len = 0;
    for(int k=0; k<2; k++)
        // To Gazebo socket, send msg of len(msg)
        msg_len = sendto(Ctrl_Mavlink_socket, msg, sizeof(msg),0, (struct sockaddr*)&addr_Mavlink, sizeof(addr_Mavlink));
    if(msg_len<0)
        cout<<"[FAILED] Ctrl_Mavlink_socket: Sending test motor speeds to Mavlink. Threads will mutual lock!"<<endl; // Not sure what mutual lock means
    else
        cout<<"[SUCCESS] Ctrl_Mavlink_socket: Sending test motor speeds to Mavlink. Avoiding mutual locking between threads!"<<endl;

    // START COMMUNCATION THREADS
    receiverThread_gazebo = std::thread(&Controller::recvThread_gazebo, this);
    receiverThread_RL = std::thread(&Controller::recvThread_RL, this);
    controllerThread = std::thread(&Controller::controlThread, this);


    // SOMETHING ABOUT QUEUEING STATES TO SEND
    queue_states = moodycamel::BlockingReaderWriterQueue<StateFull>(5);
    queue_motorspeed = moodycamel::BlockingReaderWriterQueue<MotorCommand>(5);
}




void Controller::recvThread_gazebo()
{   // Receives state array from Gazebo-Mavlink, then sends it to the RL Socket
    double state_full[18];
    StateFull state_full_structure;

    while(isRunning)
    {
        //cout<<"[recvThread_gazebo] Receiving crazyflie states from Gazebo"<<endl;
        // Receive states from Mavlink and store in state_full
        int len = recvfrom(Ctrl_Mavlink_socket, state_full, sizeof(state_full),0, (struct sockaddr*)&addr_Mavlink, &addr_Mavlink_len);

        // Take data received and copy it to state_full_structure for access outside of this function
        memcpy(state_full_structure.data, state_full, sizeof(state_full));        
        queue_states.enqueue(state_full_structure); // Not sure why we do this

              
    }
}



void Controller::recvThread_RL()
{
    float motorspeed_fake[4] = {0,0,0,0};

    while(isRunning)
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





Vector3d dehat(Matrix3d a_hat) // Input a skew-symmetric matrix and output corresponding vector
{

    /* Convert skew-symmetric matrix a_hat into vector a

    a_hat = [  0   -a3   a2 ]
            [  a3   0   -a1 ]
            [ -a2   a1   0  ]


    a = [ a1 ] 
        [ a2 ] 
        [ a3 ]

    */

    Vector3d a;
    Matrix3d tmp;

    tmp = (a_hat - a_hat.transpose())/2; // Not sure why this is done

    a(0) = tmp(2,1);
    a(1) = tmp(0,2);
    a(2) = tmp(1,0);

    return a;
}

Matrix3d hat(Vector3d a) // Input a hat vector and output corresponding skew-symmetric matrix
{ 
  // You hat a vector and get a skew-symmetric matrix
  // You dehat/dehat a skew-symmetric matrix and get a vector

    /* Convert a into skew symmetric matrix a_hat
    a = [ a1 ] 
        [ a2 ] 
        [ a3 ]
 
    a_hat = [  0   -a3   a2 ]
            [  a3   0   -a1 ]
            [ -a2   a1   0  ]
    ]
    */
    Matrix3d a_hat;
    a_hat(2,1) =  a(0);
    a_hat(1,2) = -a(0);

    a_hat(0,2) =  a(1);
    a_hat(2,0) = -a(1);

    a_hat(1,0) =  a(2);
    a_hat(0,1) = -a(2);

    return a_hat;
}














void Controller::controlThread()
{

    // =========== Controller Explanation =========== //
    // https://www.youtube.com/watch?v=A27knigjGS4&list=PL_onPhFCkVQhuPiUxUW2lFHB39QsavEEA&index=46
    // Derived from DOI's: 10.1002/asjc.567 (T. Lee) & 10.13140/RG.2.2.12876.92803/1 (M. Fernando)
    typedef Matrix<double, 3, 3, RowMajor> RowMatrix3d; 

    
    StateFull state_full_structure;
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
    Vector3d x_d_Def(0,0,0.23); // Pos-desired (Default) [m]  # Should be z=0.03 but needs integral controller for error offset
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

    


    // Matrix3d J_temp;
    // J_temp << 16.5717,0.8308,0.7183,
    //           0.8308,16.6556,1.8002,
    //           0.7183,1.8002,29.2617; // Sourced from J. Forster
    // J = J_temp*1e-6;




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
    double kp_x = 0.1;   // Pos. Gain
    double kd_x = 0.08;  // Pos. derivative Gain
    double ki_x = 0.05*0; // Pos. integral Gain
    double kp_R = 0.05;  // Rot. Gain // Keep checking rotational speed
    double kd_R = 0.006; // Rot. derivative Gain

    double kp_omega = 0.0005; 
    // Omega proportional gain (similar to kd_R but that's for damping and this is to achieve omega_d)
    // (0.0003 Fully saturates motors to get to omega_max (40 rad/s))
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
    double flip_flag = 1; // Controls thrust implementation
    double motorstop_flag = 0; // Controls stop implementation



   
    double w; // Angular frequency for trajectories [rad/s]
    double b; // Amplitude for trajectories [m]

    // =========== Trajectory Definitions =========== //
    x_d << x_d_Def;
    v_d << v_d_Def;
    a_d << a_d_Def;
    b1_d << b1_d_Def;

    while(isRunning)
    {

        
      
        // if (t>=8.1){ // Vertical Petal Traj.
        // w = 3.0; // rad/s
        // x_d << (cos(M_PI/2 + t*w)*cos(t))/2, 0, (cos(M_PI/2 + t*w)*sin(t))/2 + 1;
        // v_d << (sin(t*w)*sin(t))/2 - (w*cos(t*w)*cos(t))/2, 0, - (sin(t*w)*cos(t))/2 - (w*cos(t*w)*sin(t))/2;
        // a_d  << (sin(t*w)*cos(t))/2 + w*cos(t*w)*sin(t) + (pow(w,2)*sin(t*w)*cos(t))/2, 0, (sin(t*w)*sin(t))/2 - w*cos(t*w)*cos(t) + (pow(w,2)*sin(t*w)*sin(t))/2;
        // b1_d << 1,0,0;
        // }

        // if (t>=20){ // Horizontal Circle Traj.
        // w = 2.0; // rad/s
        // b = 0.5;
        // x_d << b*cos(t*w), b*sin(t*w), 1.5;
        // v_d << -b*w*sin(t*w), b*w*cos(t*w), 0;
        // a_d << -b*pow(w,2)*cos(t*w), -b*pow(w,2)*sin(t*w), 0;
        // b1_d << 1,0,0;
        // }



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

                kd_R = 0.005; 

                kp_x = 0.1;   // Pos. Gain
                kd_x = 0.1;  // Pos. derivative Gain
                ki_x = 0.05*0; // Pos. integral Gain
                kp_R = 0.05;  // Rot. Gain // Keep checking rotational speed
                kd_R = 0.005; // Rot. derivative Gain

                e_intg <<0,0,0; 

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
                kp_x = Ctrl_Gains(0);
                kd_x = Ctrl_Gains(1);
                // ki_x = Ctrl_Gains(2);
                kp_R = Ctrl_Gains(2);
                kd_R = Ctrl_Gains(3);
                break;
        }

        // =========== State Definitions =========== //

        // Define state_full from recieved Gazebo states and break into corresponding vectors
        queue_states.wait_dequeue(state_full_structure);
        // memcpy(state_full, state_full_structure.data, sizeof(state_full));

        Map<Matrix<double,1,18>> state_full_Eig(state_full_structure.data); // Convert threaded array to Eigen vector   
        t = state_full_Eig(0);   
        pos = state_full_Eig.segment(1,3); // .segment(index,num of positions)
        quat_Eig = state_full_Eig.segment(4,4);
        vel = state_full_Eig.segment(8,3);
        omega = state_full_Eig.segment(11,3);
        
        dt = t - t_prev;

        
        




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
        
        e_intg += e_x*dt;

        F_thrust_ideal = -kp_x*kp_xf*e_x + -kd_x*kd_xf*e_v + -ki_x*e_intg + m*g*e3 + m*a_d; // ideal control thrust vector
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
        

        if(flip_flag == 0){ // I've just sold my soul by doing this
            e_omega(2) = 0; // Remove yaw control when executing flip
        }



        // =========== Control Equations =========== // 
        F_thrust = F_thrust_ideal.dot(b3)*(flip_flag); // Thrust control value
        Gyro_dyn = omega.cross(J*omega) - J*(hat(omega)*R.transpose()*R_d*omega_d - R.transpose()*R_d*domega_d); // Gyroscopic dynamics
        M = -kp_R*e_R*(kp_Rf) + -kd_R*e_omega*(kd_Rf) + Gyro_dyn; // Moment control vector
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

        if(b3(2) <= 0 & flip_flag==0){ // If e3 component of b3 is neg, turn motors off [arbitrary amount]
            motorspeed_Vec << 0,0,0,0;
        }


        if(motorstop_flag == 1){ // Shutoff all motors
            motorspeed_Vec << 0,0,0,0;
        }
        


        // if (t_step%75 == 0){ // General Debugging output
        // cout << setprecision(4) <<
        // "t: " << t << "\tCmd: " << control_cmd_Eig.transpose() << endl <<
        // "kp_x: " << kp_x << " \tkd_x: " << kd_x << " \tkp_R: " << kp_R << " \tkd_R: " << kd_R << "\tkd_R_fl: " << kp_omega << endl <<
        // "kp_xf: " << kp_xf << " \tkd_xf: " << kd_xf << " \tkp_Rf: " << kp_Rf << " \tkd_Rf: " << kd_Rf << " \tflip_flag: " << flip_flag << endl <<
        // "ki_x: " << ki_x  << "\t e_x_I: " << e_intg.transpose() <<
        // endl <<
        // "x_d: " << x_d.transpose() << endl <<
        // "v_d: " << v_d.transpose() << endl <<
        // "omega_d: " << omega_d.transpose() << endl <<
        // endl << 
        // "pos: " << pos.transpose() << "\tex: " << e_x.transpose() << endl <<
        // "vel: " << vel.transpose() << "\tev: " << e_v.transpose() << endl <<
        // "omega: " << omega.transpose() << "\te_w: " << e_omega.transpose() << endl <<
        // endl << 
        // "R:\n" << R << "\n\n" << 
        // "R_d:\n" << R_d << "\n\n" << 
        // "Yaw: " << yaw*180/M_PI << "\tRoll: " << roll*180/M_PI << "\tPitch: " << pitch*180/M_PI << endl <<
        // "e_R: " << e_R.transpose() << "\te_R (deg): " << e_R.transpose()*180/M_PI << endl <<
        // endl <<
        // "FM: " << FM.transpose() << endl <<
        // "f: " << f.transpose() << endl <<
        // endl << setprecision(0) <<
        // "MS_d: " << motorspeed_Vec_d.transpose() << endl <<
        // "MS: " << motorspeed_Vec.transpose() << endl <<
        // "=============== " << endl; 
        // printf("\033c"); // clears console window
        // }

     
        Map<RowVector4f>(&motorspeed[0],1,4) = motorspeed_Vec.cast <float> (); // Converts motorspeeds to C++ array for data transmission
        int len = sendto(Ctrl_Mavlink_socket, motorspeed, sizeof(motorspeed),0, // Send motorspeeds to Gazebo -> gazebo_motor_model?
                (struct sockaddr*)&addr_Mavlink, addr_Mavlink_len); 
        

        t_step++;
        t_prev = t;

    }
}



int main()
{
    Controller controller;
    controller.Load(); // Run controller as a thread

    while(1)
    {
        sleep(1e7);         // 1e7 is about 100 days
    }
    
    return 0;
}