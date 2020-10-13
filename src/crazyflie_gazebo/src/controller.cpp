#include "controller.h"
#include "math_linear_algebra.h"
#include <iomanip>      // provides std::setprecision
#include <math.h>
#include <algorithm>
#include <stdint.h>
#include <Eigen/Dense>

using namespace Eigen;
using namespace std;

void Controller::Load(int port_number_gazebo)
{
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

        /*if(len>0)
        {
            cout<<"Enqueue control command: ";
            for(int k=0;k<5;k++)
                cout<<control_cmd_recvd[k]<<", ";
            cout<<"\n";
        }*/
        if(control_cmd_recvd[0]>10)
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
    double orientation_q[4];

    Vector3d pos; // current position [m]
    Vector3d vel; // current velocity [m]
    Vector4d quat_Eig; // current attitude [rad] (quat form)
    Vector3d eul_d; // current attitude [rad] (roll, pitch, yaw angles)
    Vector3d omega; // current angular velocity [rad/s]

    
    double R[3][3];

    
    // State Error and Prescriptions
    Vector3d x_d; // Pose-desired [m] 
    Vector3d v_d; // Velocity-desired [m/s]
    Vector3d a_d(0,0,0); // Acceleration-desired [m/s]

    Matrix3d R_d; // Rotation-desired (pitch, roll, yaw euler angles)
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

    Matrix4d Gamma_inv; // Calculated by Matlab but not sure what it is
    Gamma_inv << 0.25, -7.6859225, -7.6859225, -41.914296,
                     0.25,  7.6859225,  7.6859225, -41.914296,
                     0.25,  7.6859225, -7.6859225,  41.914296,
                     0.25, -7.6859225,  7.6859225,  41.914296;


    Matrix4d Gamma;


    // gamma = np.mat( [[1,1,1,1], [-d,d,d,-d], [-d,d,-d,d], [-c_tau,-c_tau,c_tau,c_tau]] )

    Vector3d e3(0,0,1); // Global z-axis

    

    Vector3d b1_d; // Desired yaw direction of body-fixed axis (COM to prop #1)
    Vector3d b2_d; // Desired body-fixed axis normal to b1 and b3
    Vector3d b3_d; // Desired body-fixed vertical axis


    Vector3d f_thrust_ideal; // Ideal thrust vector to minimize error   
    Vector3d tau; // Moment control vector
    
    Vector4d FT;
    Vector4d f;

    

    Vector4d motorspeed_square;
    Vector4d motorspeed_Eig; // motorspee

    Vector3d b3; // body-fixed vertical axis
    


    


    double t;


    


    // Controller Values
    // only kp_v and kp_R12 matter for vel control
    double kp_x = 0.15; // Positional Gain
    double kp_v = 3.25; // Velocity Gain

    double kp_R12 = 0.55;// Kp_R
    double kd_R12 = 0.1; // Kd_R
    double kp_R34 = 1e-5; // Are these for roll and pitch?
    double kd_R34 = 5e-4;

    double kp_R = 1e-5;
    double kd_R = 5e-4;
    double kd_R2 = 1e-6;

    double c_T = 1.2819184e-8; // Motor constant


    double f_thrust = 0;
    // might need to adjust weight to real case (sdf file too)

    double m = 0.026 + 0.00075*4; // Mass [kg]
    double g = 9.8066; // Gravitational acceleration [m/s^2]
    double f_hover = m*g; // Force to hover


    double d; //= ___; // distance from COM to prop
    double c_Tf;// = _____ // Ratio between km and kf (Not sure what these correspond to)
    Gamma << 1,    1,     1,    1,
                 0,   -d,     0,    d,
                 d,    0,    -d,    0,
                -c_Tf, c_Tf, -c_Tf, c_Tf;
 

  


    unsigned int k_run = 0; // Run counter

    while(isRunning_)
    {
        k_run++;

        // Define state_full from recieved Gazebo states and break into corresponding vectors
        queue_states_.wait_dequeue(state_full_structure);
        // memcpy(state_full, state_full_structure.data, sizeof(state_full));

        Map<Matrix<double,1,14>> state_full_Eig(state_full_structure.data); // Convert threaded array to Eigen vector     
        pos = state_full_Eig.segment(0,3); // .segment(index,num of positions)
        quat_Eig = state_full_Eig.segment(3,4);
        vel = state_full_Eig.segment(7,3);
        omega = state_full_Eig.segment(10,3);
        t = state_full_Eig(13);
        

    

        // Define control_cmd from recieved control_cmd
        if (control_cmd_recvd[0]<10) // There is a case where control_cmd_recvd becomes 11 but I'm not sure why?
            memcpy(control_cmd, control_cmd_recvd, sizeof(control_cmd)); // Compiler doesn't work without this line for some reason? 
            Map<Matrix<double,5,1>> control_cmd_Eig(control_cmd_recvd); 



        type = control_cmd_Eig(0); // Command type
        control_vals = control_cmd_Eig.segment(1,3); // Command values
        ctrl_flag = control_cmd_Eig(4); // Controller On/Off switch (To be implemented)



        // Quaternion to Rotation Matrix Conversion
        Quaterniond q;
        q.w() = quat_Eig(0);
        q.vec() = quat_Eig.segment(1,3);
        Matrix3d R_Eig = q.normalized().toRotationMatrix(); 
        // I'm not sure if this from Body->World or World->Body
        


        if (type == 1 || type == 2)
        {
            if (type == 1) // position error calc 
            {   
                
                x_d = control_vals; // Set desired position from thread
                a_d << 0,0,0; // Set desired acceleration from thread (default to zero for our typical use)

                // x_d = _______ // Set desired position from a path function e.g. [cos(pi*t),sin(pi*t),2]
                // v_d = ______ // Set desired velocity from a path function e.g. [-pi*sin(pi*t),pi*cos(pi*t),0]
                // a_d = ______ // Set desired acceleration from a path function e.g. [-pi^2*cos(pi*t),-pi^2*sin(pi*t),0]

                e_x = pos - x_d; //  Position Error
            }
            else if (type == 2) // velocity error calc
            {   
                v_d = control_vals; // velocity desired
                e_v = vel - v_d; // velocity error 
            }

            e_x << 0,0,0; // Errors will need to be set to zero when not being used ============



            // =========== Calculate the prescribed thrust =========== //  
            // 
            // This is in terms of the global axes [x,y,z] => [e1,e2,e3] 
            // 
            // Calculates total error in 3D space and the required thrust vector to exponentially minimize it
            // but because thrust is locked in b3 direction, while error vector can point anywhere,
            // we project the vector onto b3 to get "close" and then use the moments to align it
            //
            // https://www.youtube.com/watch?v=A27knigjGS4&list=PL_onPhFCkVQhuPiUxUW2lFHB39QsavEEA&index=46

            b3 = R_Eig.col(2); // current orientation of b3 vector
            f_thrust_ideal = -kp_x*e_x + -kp_v*e_v + m*g*e3 - m*a_d; // thrust error vector
            f_thrust = f_thrust_ideal.dot(b3); // ideal thrust projected onto b3

            // If the prescribed thrust is globally z-negative then turn thrust 
            // off so it doesn't dive bomb in a fiery explosion of death (or break)
            if (f_thrust_ideal(2)<0)
                f_thrust_ideal(2) = 0.01;

            

            // =========== Calculate desired body-fixed axes =========== //
            // Defines the desired yaw angle of CF (Facing positive x-axis)

            b1_d << 1,0,0;  // b1 is unit basis vector from COG to propeller (one?)
            b3_d= f_thrust_ideal.normalized(); // body-fixed vertical axis

            b2_d = b3_d.cross(b1_d); // body-fixed axis to prop (2?)
            b2_d.normalize();
            

            // =========== Calculate Rotational Error Matrix =========== // 
            R_d << b1_d, b2_d, b3_d; // concatinating column vectors of desired body axes
            e_R = dehat(0.5*(R_d.transpose()*R_Eig - R_Eig.transpose()*R_d));



            // Just fix code so omega_d = zero          
            // This is a trick to have because omega_d set = 0 so e_omega = omega-omega_d ======================= 
            e_omega = omega; // This is wrong way and purely temporary to keep consistent with the current controller =============


            // =========== Calculate Moment Vector (tau or M) =========== //
            tau = -kp_R12*e_R + -kd_R12*e_omega + omega.cross(J*omega) 
                    + J*(hat(omega_d)*R_Eig.transpose()*R_d*omega_d - R_Eig.transpose()*R_d*domega_d); 


        } 
        else if (type == 3 || type==4)
        {
            if (type == 3) // attitude control
            {
                eul_d = control_vals;

                R_d  <<  cos(eul_d(1)),  0,  sin(eul_d(1)), // This is locking us in only pitch ==========
                             0,            1,  0,
                            -sin(eul_d(1)),  0,  cos(eul_d(1));

                e_R = dehat(0.5*(R_d.transpose()*R_Eig - R_Eig.transpose()*R_d));
                e_omega = omega; // This is the wrong way and purely temporary to keep consistent with the current controller =============
            }
            else if (type == 4)// Angular velocity control
            {
                omega_d = control_vals;

                omega_d(0) = omega(0) + omega_d(0); // I can't quite follow this ================
                omega_d(2) = omega(2) + omega_d(2);
                
                e_R << 0,0,0;
                e_omega = omega - omega_d;
            }

            if (R_Eig(2,2) > 0.7) // If pitch angle is > 45 deg then divide by cos of pitch angle? ===========
                f_thrust = f_hover/R_Eig(2,2);
            else 
                f_thrust = f_hover/0.7; // Otherwise divide hover

            tau = -kp_R34*e_R + -kd_R34*e_omega + omega.cross(J*omega); 
        }
        else if (type == 5)// If command[0] = 5 stop all thrust
        {
            f_thrust = 0;
            tau << 0,0,0;
            
        }
        

        FT << f_thrust, tau; // Controller prescribed thrust and moments
        f = Gamma_inv*FT; // Eq.5 - Convert prescribed thrust and moments to individual motor thrusts
        motorspeed_square = f/c_T; // 
        
        

        // If squared motorspeed^2 is negative cap at zero
        // Why that'd be the case? I don't know
        for(int k_motor=0;k_motor<4;k_motor++)
        {
            if(motorspeed_square(k_motor)<0){
                motorspeed_square(k_motor) = 0;}
            else if (isnan(motorspeed_square(k_motor))){
               motorspeed_square(k_motor) = 0;}
        }
        
 

        if(R_Eig(2,2)<0) // If pitch angle goes less than 90 deg then shut off motors
        {
            motorspeed_square(0) = 0;
            motorspeed_square(1) = 0;
            motorspeed_square(2) = 0;
            motorspeed_square(3) = 0;
        }

        motorspeed_Eig = motorspeed_square.array().sqrt();
        Map<RowVector4f>(&motorspeed[0],1,4) = motorspeed_Eig.cast <float> (); // Converts motorspeeds to C++ array for data transmission


        // double ms_min = 0.0;
        // double ms_max = 3420.0;
        // for (int i =0; i<4;i++) { // clamp motor speed (based on max thrust (0.6) speed)
        //     motorspeed[i] = math::clamp(motorspeed[i],ms_min,ms_max);
        // }

        // cout causing wierd behavior?????
        //if ( k_run%50 == 1 ) {
        //        cout<<"motor speed ["<< motorspeed[0]<<", "<< motorspeed[1]<<", "<< motorspeed[2]<<", "<<motorspeed[3]<<"]"<<endl;
        //}   
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