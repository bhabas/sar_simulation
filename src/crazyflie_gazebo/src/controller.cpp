#include "controller.h"
#include <iomanip>      // provides std::setprecision
#include <math.h>
#include <algorithm>
#include <stdint.h>
#include <Eigen/Dense>

using namespace Eigen;
using namespace std;


void Controller::Load(int port_number_gazebo)
{   cout << setprecision(3);
    fd_gazebo_ = socket(AF_INET, SOCK_DGRAM, 0);
    fd_gazebo_SNDBUF_ = 16;         // 16 bytes is 4 float
    fd_gazebo_RCVBUF_ = 112;        // 112 bytes is 14 double
    if (setsockopt(fd_gazebo_, SOL_SOCKET, SO_SNDBUF, &fd_gazebo_SNDBUF_, sizeof(fd_gazebo_SNDBUF_))<0)
        cout<<"fd_gazebo_ setting SNDBUF failed"<<endl;
    if (setsockopt(fd_gazebo_, SOL_SOCKET, SO_RCVBUF, &fd_gazebo_RCVBUF_, sizeof(fd_gazebo_RCVBUF_))<0)
        cout<<"fd_gazebo_ setting RCVBUF failed"<<endl;
    port_number_gazebo_ = port_number_gazebo;

    memset(&sockaddr_local_gazebo_, 0, sizeof(sockaddr_local_gazebo_));
    sockaddr_local_gazebo_.sin_family = AF_INET;
    sockaddr_local_gazebo_.sin_addr.s_addr = htonl(INADDR_ANY);//inet_addr("127.0.0.1");
    sockaddr_local_gazebo_.sin_port = htons(18070);
    
    if (bind(fd_gazebo_, (struct sockaddr*)&sockaddr_local_gazebo_, sizeof(sockaddr_local_gazebo_))<0)
        cout<<"Socket binding to Gazebo failed"<<endl;
    else
        cout<<"Socket binding to Gazebo succeeded"<<endl; 

    memset(&sockaddr_remote_gazebo_, 0, sizeof(sockaddr_remote_gazebo_));
    sockaddr_remote_gazebo_.sin_family = AF_INET;
    sockaddr_remote_gazebo_.sin_addr.s_addr = htonl(INADDR_ANY);
    sockaddr_remote_gazebo_.sin_port = htons(port_number_gazebo_);
    sockaddr_remote_gazebo_len_ = sizeof(sockaddr_remote_gazebo_);
    
    float buf[4] = {100.0,100.0,100.0,100.0};
    int len = 0;
    for(int k=0; k<2; k++)
        len = sendto(fd_gazebo_, buf, sizeof(buf),0, (struct sockaddr*)&sockaddr_remote_gazebo_, sizeof(sockaddr_remote_gazebo_));
    if(len>0)
        cout<<"Send initial motor speed ["<<len<<" bytes] to Gazebo Succeeded! \nAvoiding threads mutual locking"<<endl;
    else
        cout<<"Send initial motor speed to Gazebo FAILED! Threads will mutual lock"<<endl;

    fd_rl_ = socket(AF_INET, SOCK_DGRAM, 0);
    fd_rl_SNDBUF_ = 112;        // 112 bytes is 14 double
    fd_rl_RCVBUF_ = 40;         // 40 bytes is 5 double
    if (setsockopt(fd_rl_, SOL_SOCKET, SO_SNDBUF, &fd_rl_SNDBUF_, sizeof(fd_rl_SNDBUF_))<0)
        cout<<"fd_rl_ setting SNDBUF failed"<<endl;
    if (setsockopt(fd_rl_, SOL_SOCKET, SO_RCVBUF, &fd_rl_RCVBUF_, sizeof(fd_rl_RCVBUF_))<0)
        cout<<"fd_rl_ setting RCVBUF failed"<<endl;
    memset(&sockaddr_local_rl_, 0, sizeof(sockaddr_local_rl_));
    sockaddr_local_rl_.sin_family = AF_INET;
    sockaddr_local_rl_.sin_addr.s_addr = htonl(INADDR_ANY);//inet_addr("127.0.0.1");
    sockaddr_local_rl_.sin_port = htons(18060);
    if (bind(fd_rl_, (struct sockaddr*)&sockaddr_local_rl_, sizeof(sockaddr_local_rl_))<0)
        cout<<"Socket binding to crazyflie_env.py failed"<<endl;
    else
        cout<<"Socket binding to crazyflie_env.py succeeded"<<endl;
    
    isRunning_ = true;
    receiverThread_gazebo_ = std::thread(&Controller::recvThread_gazebo, this);
    //senderThread_gazebo_ = std::thread(&Controller::sendThread_gazebo, this);
    receiverThread_rl_ = std::thread(&Controller::recvThread_rl, this);
    controllerThread_ = std::thread(&Controller::controlThread, this);

    queue_states_ = moodycamel::BlockingReaderWriterQueue<StateFull>(5);
    queue_motorspeed_ = moodycamel::BlockingReaderWriterQueue<MotorCommand>(5);
}
















void Controller::recvThread_gazebo()
{
    double state_full[14];
    StateFull state_full_structure;

    while(isRunning_)
    {
        //cout<<"[recvThread_gazebo] Receiving crazyflie states from Gazebo"<<endl;
        int len = recvfrom(fd_gazebo_, state_full, sizeof(state_full),0, (struct sockaddr*)&sockaddr_remote_gazebo_, &sockaddr_remote_gazebo_len_);

        /*if(len>0)
        {
            cout<<"Enqueue full State:";
            for(int k=0;k<13;k++)
                cout<<state_full[k]<<", ";
            cout<<"\n";
        }*/

        memcpy(state_full_structure.data, state_full, sizeof(state_full));
        queue_states_.enqueue(state_full_structure);
        sendto(fd_rl_, state_full, sizeof(state_full),0, (struct sockaddr*)&sockaddr_remote_rl_, sockaddr_remote_rl_len_);
    }
}


void Controller::sendThread_gazebo()
{
    float motorspeed[4];
    MotorCommand motorspeed_structure;

    while(isRunning_)
    {
        queue_motorspeed_.wait_dequeue(motorspeed_structure);
        memcpy(motorspeed, motorspeed_structure.data, sizeof(motorspeed));

        //cout<<"[recvThread_rl] sending motor speed to Gazebo"<<endl;
        int len=sendto(fd_gazebo_, motorspeed, sizeof(motorspeed),0, (struct sockaddr*)&sockaddr_remote_gazebo_, sockaddr_remote_gazebo_len_);
        if(len>0)
            cout<<"[recvThread_rl] sent motor speed ["<<motorspeed[0]<<", "<<motorspeed[1]<<", "<<motorspeed[2]<<", "<<motorspeed[3]<<"]"<<endl;
        else
            cout<<"[recvThread_rl] sending motor speed to Gazebo FAILED!"<<endl;
    }
    
}


















void Controller::recvThread_rl()
{
    float motorspeed_fake[4] = {0,0,0,0};

    while(isRunning_)
    {
        //cout<<"[recvThread_rl] Receiving command from RL"<<endl;
        int len = recvfrom(fd_rl_, control_cmd_recvd, sizeof(control_cmd_recvd),0, (struct sockaddr*)&sockaddr_remote_rl_, &sockaddr_remote_rl_len_);


        if(control_cmd_recvd[0]>10) // If header is 11 then enable sticky
        {
            motorspeed_fake[0] = -control_cmd_recvd[0];
            motorspeed_fake[1] = control_cmd_recvd[1];
            //cout<<"Send sticky command command: "<< motorspeed_fake[0]<<", "<<motorspeed_fake[1]<<endl;
            sendto(fd_gazebo_, motorspeed_fake, sizeof(motorspeed_fake),0, (struct sockaddr*)&sockaddr_remote_gazebo_, sockaddr_remote_gazebo_len_);

            /*if (control_cmd_recvd[1]<0.5)     // reset_world signal
            {
                control_cmd_recvd[0] = 2; control_cmd_recvd[1] = 0; control_cmd_recvd[2] = 0; control_cmd_recvd[3] = 0; control_cmd_recvd[4] = 0;
            }*/
            
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
    typedef Matrix<double, 3, 3, RowMajor> RowMatrix3d; 

    double state_full[14];
    StateFull state_full_structure;
    float motorspeed[4];
    MotorCommand motorspeed_structure;

    int type = 5; // Command type {1:Pos, 2:Vel, 3:Att, 4:Omega, 5:Stop}
    int ctrl_flag; // On/Off switch for controller
    double control_cmd[5];
    Vector3d control_vals;
    

    // State Declarations

    Vector3d pos; // current position [m]
    Vector3d vel; // current velocity [m]
    Vector4d quat_Eig; // current attitude [rad] (quat form)
    Vector3d eul_d; // current attitude [rad] (roll, pitch, yaw angles)
    Vector3d omega; // current angular velocity [rad/s]

    

    
    // State Error and Prescriptions
    Vector3d x_d; // Pose-desired [m] 
    Vector3d v_d; // Velocity-desired [m/s]
    Vector3d a_d(0,0,0); // Acceleration-desired [m/s]

    Matrix3d R_d; // Rotation-desired 
    Matrix3d R_d2; // Rotation-desired (ZXY Euler angles)
    Vector3d omega_d; // Omega-desired
    Vector3d domega_d(0,0,0);

    Vector3d e_x; // Pose-Error
    Vector3d e_v; // Vel-error 
    Vector3d e_R; // Rotation-error
    Vector3d e_omega; // Omega-error

    

    Matrix3d J; // Rotational Inertia of CF
    J<< 1.65717e-05, 0, 0,
        0, 1.66556e-05, 0,
        0, 0, 2.92617e-05;


    Matrix4d Gamma;


    Vector3d e3(0,0,1); // Global z-axis

    

    Vector3d b1_d; // Desired yaw direction of body-fixed axis (COM to prop #1)
    Vector3d b2_d; // Desired body-fixed axis normal to b1 and b3
    Vector3d b3_d; // Desired body-fixed vertical axis


    Vector3d F_thrust_ideal; // Ideal thrust vector to minimize error   
    Vector3d M; // Moment control vector
    
    Vector4d FM; // Thrust & Moment control vector (4x1)
    Vector4d f;

    

    Vector4d motorspeed_square;
    Vector4d motorspeed_Eig; // motorspee

    Vector3d b3; // body-fixed vertical axis
    

    Quaterniond q;
    Matrix3d R;
    Vector3d eul;


    


    


    double t;


    


    // Controller Values

    double kp_x = 1.0; // Positional Gain
    double kd_x = 0.658; // Velocity Gain
    double kp_R = -0.0055*0;// Kp_R
    double kd_R = 0; //0.036e4; // Kd_R

    double kp_xf = 1; // Positional Gain Flag
    double kd_xf = 1; // Velocity Gain Flag
    double kp_Rf = 1; // Attitude Gain Flag
    double kd_Rf = 1; // Anglular Rate Gain Flag


    double c_T = 1.2819184e-8; // Motor constant


    double F_thrust = 0;
    // might need to adjust weight to real case (sdf file too)

    double m = 0.026 + 0.00075*4; // Mass [kg]
    double g = 9.8066; // Gravitational acceleration [m/s^2]
    double f_hover = m*g; // Force to hover


    double d = 0.040; // distance from COM to prop
    double d_p = d*sin(M_1_PI/4);
    // double c_Tf = 0.006; // Ratio between km and kf (Not sure what these correspond to)
    double c_Tf = 0.0037;
    c_Tf = 1.2819184e-8;
    
    Gamma <<1,     1,     1,     1,
            d_p,   d_p,  -d_p,  -d_p, 
            d_p,  -d_p,  -d_p,   d_p, 
            c_Tf, -c_Tf,  c_Tf, -c_Tf;
 

  
    unsigned int k_run = 0; // Run counter


    // =========== Trajectory Definitions =========== //
    x_d << 0,0,0;
    v_d << 0,0,0;
    a_d << 0,0,0;
    b1_d << 1,0,0;


    while(isRunning_)
    {
        k_run++;

        // =========== State Definitions =========== //

        // Define state_full from recieved Gazebo states and break into corresponding vectors
        queue_states_.wait_dequeue(state_full_structure);
        // memcpy(state_full, state_full_structure.data, sizeof(state_full));

        Map<Matrix<double,1,14>> state_full_Eig(state_full_structure.data); // Convert threaded array to Eigen vector     
        pos = state_full_Eig.segment(0,3); // .segment(index,num of positions)
        quat_Eig = state_full_Eig.segment(3,4);
        vel = state_full_Eig.segment(7,3);
        omega = state_full_Eig.segment(10,3);
        t = state_full_Eig(13); 
        

    
        // =========== Control Definitions =========== //
        // Define control_cmd from recieved control_cmd
        if (control_cmd_recvd[0]<10) // 
            memcpy(control_cmd, control_cmd_recvd, sizeof(control_cmd)); // Compiler doesn't work without this line for some reason? 
            Map<Matrix<double,5,1>> control_cmd_Eig(control_cmd_recvd); 



        type = control_cmd_Eig(0); // Command type
        control_vals = control_cmd_Eig.segment(1,3); // Command values
        ctrl_flag = control_cmd_Eig(4); // Controller On/Off switch (To be implemented)



        switch(type){ // Define Desired Values
            case 1: // Position
                x_d << control_vals;
                if (ctrl_flag == 0) kp_xf = 0;
                break;
            case 2: // Velocity
                v_d << control_vals;
                if (ctrl_flag == 0) kd_xf = 0;
                break;
            case 3: // Attitude [Implementation needs to be finished]
                // How to work off R_d2 only when needed?
                eul_d << control_vals;
                // R_d2 = AngleAxisf(eul_d(2), Vector3f::UnitZ()) // Yaw
                //         * AngleAxisf(eul_d(0), Vector3f::UnitX()) // Roll
                //         * AngleAxisf(eul_d(1), Vector3f::UnitY()); // Pitch
                if (ctrl_flag == 0) kp_Rf = 0;
                break;
            case 4: // Ang. Velocity
                omega_d << control_vals;
                if (ctrl_flag == 0) kd_Rf = 0;
                break;
            case 5: // Stop Motors
                break;
        }

 




        // =========== Rotation Matrix =========== //
        // I'm not sure if this from Body->World or World->Body
        // https://www.andre-gaschler.com/rotationconverter/
        q.w() = quat_Eig(0);
        q.vec() = quat_Eig.segment(1,3);
        R = q.normalized().toRotationMatrix(); // Quaternion to Rotation Matrix Conversion
        eul = R.eulerAngles(2,0,1); // Euler angles ZXY
        b3 = R*e3;


        // =========== Translational Errors & Desired Body-Fixed Axes =========== //
        e_x = pos - x_d;
        e_v = vel - v_d;

        F_thrust_ideal = -kp_x*kp_xf*e_x + -kd_x*kd_xf*e_v + m*g*e3 + m*a_d; // ideal control thrust vector
        b3_d = F_thrust_ideal.normalized(); // desired body-fixed vertical axis
        b2_d = b3_d.cross(b1_d).normalized(); // body-fixed horizontal axis



        // =========== Rotational Errors =========== // 
        R_d << b2_d.cross(b3_d).normalized(),b2_d,b3_d; // Desired rotational axis
                                                        // b2_d x b3_d != b1_d (look at derivation)
        e_R = 0.5*dehat(R_d.transpose()*R - R.transpose()*R_d); // Rotational error
        e_omega = omega - R.transpose()*R_d*omega_d; // Angular vel error




        // =========== Control Equations =========== // 
        F_thrust = F_thrust_ideal.dot(b3); // Thrust control value
        M = -kp_R*kp_Rf*e_R + -kd_R*kd_Rf*e_omega + omega.cross(J*omega) // Moment control vector
                - J*(hat(omega)*R.transpose()*R_d*omega_d - R.transpose()*R_d*domega_d);
        FM << F_thrust,M; // Thrust-Moment control vector



        // =========== Propellar Thrusts/Speeds =========== //
        f = Gamma.inverse()*FM; // Propeller thrusts
        motorspeed_square = (f*1/c_Tf);
        motorspeed_Eig = motorspeed_square.array().sqrt();


        // If CF overshoots it will prescribe a negative 
        // force which it can't do so we cap the command at zero 
        // (potentially higher to reduce startup time)
        for(int k_motor=0;k_motor<4;k_motor++) 
        {
            if(motorspeed_Eig(k_motor)<0){
                motorspeed_Eig(k_motor) = 0;}
            else if (isnan(motorspeed_Eig(k_motor))){
                motorspeed_Eig(k_motor) = 0;}
        }
        

        // cout << e_R.transpose() << " | " << pos(2) << "\t" <<eul.transpose() << endl;
        // cout << R_d.transpose()*R - R.transpose()*R_d << endl << endl;
        cout << e_R.transpose() << " | " << 0.5*dehat(R_d.transpose()*R - R.transpose()*R_d).transpose() << endl << endl;

        // cout << motorspeed_Eig.transpose() << " | " << pos(2) << endl << endl;

        
        Map<RowVector4f>(&motorspeed[0],1,4) = motorspeed_Eig.cast <float> (); // Converts motorspeeds to C++ array for data transmission
        sendto(fd_gazebo_, motorspeed, sizeof(motorspeed),0, (struct sockaddr*)&sockaddr_remote_gazebo_, sockaddr_remote_gazebo_len_);

    }
}



int main()
{
    Controller controller;
    
    controller.Load(18080);


    while(1)
    {
        sleep(1e7);         // 1e7 is about 100 days
    }
    
    return 0;
}