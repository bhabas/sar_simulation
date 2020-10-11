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
        cout<<"Send initial motor speed ["<<len<<" bytes] to Gazebo Succeeded! Avoiding threads mutual locking"<<endl;
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

    double control_cmd[5];
    int type = 5;
    double control_vals[3];
    Vector3d control_vals_Eig;

    // State Declarations

    double orientation_q[4];
    double eul[3];


    // Rotation Matrices
    double R[3][3];



    Vector3d x_d_Eig; // Pose-desired
    Vector3d e_x_Eig; // Pose-Error

    Vector3d v_d_Eig; // Vel-desired
    Vector3d e_v_Eig; // Vel-error

    Matrix3d R_d_Eig; // Rotation-desired
    Vector3d e_R_Eig; // Rotation-error

    Vector3d omega_d_Eig; // Omega-desired
    Vector3d e_omega_Eig; // Omega-error

    Vector3d tau_Eig;
    Matrix3d J_Eig;
    J_Eig<< 1.65717e-05, 0, 0,
        0, 1.66556e-05, 0,
        0, 0, 2.92617e-05;

    Matrix4d Gamma_inv_Eig; // Calculated by Matlab but not sure what it is
    Gamma_inv_Eig << 0.25, -7.6859225, -7.6859225, -41.914296,
                     0.25,  7.6859225,  7.6859225, -41.914296,
                     0.25,  7.6859225, -7.6859225,  41.914296,
                     0.25, -7.6859225,  7.6859225,  41.914296;

    // gamma = np.mat( [[1,1,1,1], [-d,d,d,-d], [-d,d,-d,d], [-c_tau,-c_tau,c_tau,c_tau]] )

    Vector3d e3_Eig(0,0,1);
    Vector3d f_total_thrust_Eig; // total thrust        

    Vector3d b1_d_Eig; // b1d: desired direction of body fixed axis in parametric form
    Vector3d b2_d_Eig;
    Vector3d b3_d_Eig;

    Vector4d FT_Eig;
    Vector4d f_Eig;
    Vector4d motorspeed_square_Eig;
    Vector4d motorspeed_Eig;

    Vector3d b3_Eig;
    Vector3d eul_Eig;

    Vector3d pos_Eig;
    Vector4d att_Eig;
    Vector3d vel_Eig;
    Vector3d omega_Eig;

    double t;


    


    // CONTROLLER GAINS
    // only kp_v and kp_R12 matter for vel control
    double kp_x = 0.15;
    double kd_x = 0.1;

    double kp_v = 3.25;

    double kp_R12 = 0.55;// Kp_R
    double kd_R12 = 0.1; // Kd_R

    double kp_R34 = 1e-5;
    double kd_R34 = 5e-4;

    double kp_R = 1e-5;
    double kd_R = 5e-4;

    double kd_R2 = 1e-6;

    double c_T = 1.2819184e-8; // Motor constant


    double f_thrust =0;
    // might need to adjust weight to real case (sdf file too)

    double m = 0.026 + 0.00075*4; // Mass [kg]
    double g = 9.8066; // Gravitational acceleration [m/s^2]
    double f_hover = m*g; // Force to hover
 

  



   




    unsigned int k_run = 0;

    while(isRunning_)
    {
        k_run++;

        // Define state_full from recieved Gazebo states and break into corresponding vectors
        queue_states_.wait_dequeue(state_full_structure);
        memcpy(state_full, state_full_structure.data, sizeof(state_full));

        Map<Matrix<double,1,14>> state_full_Eig(state_full_structure.data);      
        pos_Eig = state_full_Eig.segment(0,3);
        att_Eig = state_full_Eig.segment(3,4);
        vel_Eig = state_full_Eig.segment(7,3);
        omega_Eig = state_full_Eig.segment(10,3);
        t = state_full_Eig(13);
    

        // Define control_cmd from recieved control_cmd
        if (control_cmd_recvd[0]<10) // There is a case where control_cmd_recvd becomes 11 but I'm not sure why?
            memcpy(control_cmd, control_cmd_recvd, sizeof(control_cmd)); // Compiler doesn't work without this line for some reason? 
            Map<Matrix<double,5,1>> control_cmd_Eig(control_cmd_recvd); 

        type = control_cmd_Eig(0);
        control_vals_Eig = control_cmd_Eig.segment(1,3);

        memcpy(orientation_q, state_full+3,  sizeof(orientation_q));





        // Extract current rotation matrix from quaternion data (Still needs to be brought into Eigen Lib)====================
        // Map<RowVector4d>(&orientation_q[0],1,3) = att_Eig;
        math::quat2rotm_Rodrigue((double *) R, orientation_q);
        Map<RowMatrix3d> R_Eig(&R[0][0]);

        Vector3d ddx_d_Eig;

        if (type == 1 || type == 2)
        {
            if (type == 1) // position error calc 
            {   
                
                x_d_Eig = control_vals_Eig; // Set desired position from thread
                // ddx_d_Eig << 0,0,0; // Set desired acceleration from thread (default to zero for our typical use)

                // x_d_Eig = _______ // Set desired position from a path function e.g. [cos(pi*t),sin(pi*t),2]
                // ddx_d_Eig = ______ // Set desired acceleration from a path function e.g. [-pi^2*cos(pi*t),-pi^2*sin(pi*t),0]

                e_x_Eig = pos_Eig - x_d_Eig; //  Position Error
            }
            else if (type == 2) // velocity error calc
            {   
                v_d_Eig = control_vals_Eig; // velocity desired
                e_v_Eig = vel_Eig - v_d_Eig; // velocity error 
            }

            e_x_Eig << 0,0,0; // Errors will need to be set to zero when not being used ============



            // =========== Calculate the Total Thrust (f) =========== //  

            f_total_thrust_Eig = -kp_x*e_x_Eig + -kp_v*e_v_Eig + f_hover*e3_Eig; // This is in terms of the global axes [x,y,z] => [e1,e2,e3] 
            

            // If the prescribed thrust is globally z-negative then turn thrust 
            // off so it doesn't dive bomb in a fiery explosion of death (or break)
            if (f_total_thrust_Eig(2)<0)
                f_total_thrust_Eig(2) = 0.01;

            

            // =========== Calculate desired body fixed axes =========== //

            b1_d_Eig << 1,0,0; // Defines the desired orientation of CF (Facing positive x-axis)
            // b1 is unit basis vector from COG to propeller (one?)
            b3_d_Eig = f_total_thrust_Eig.normalized(); 
            b2_d_Eig = b3_d_Eig.cross(b1_d_Eig);
            b2_d_Eig.normalize();
            
            // b1_d_Eig = b2_d_Eig.cross(b3_d_Eig); // Not sure why Pan redefined this axis? =============
            // b1_d_Eig.normalize(); 



            // =========== Calculate Rotational Error Matrix =========== // 
            R_d_Eig << b1_d_Eig, b2_d_Eig, b3_d_Eig; // concatinating column vectors of desired body axes
            e_R_Eig = dehat(0.5*(R_d_Eig.transpose()*R_Eig - R_Eig.transpose()*R_d_Eig));



            // Just fix code so omega_d = zero          
            // This is a trick to have because omega_d set = 0 so e_omega = omega-omega_d ======================= 
            e_omega_Eig = omega_Eig; // This is wrong way and purely temporary to keep consistent with the current controller =============


            // =========== Calculate Moment Vector (tau or M) =========== //
            tau_Eig = -kp_R12*e_R_Eig + -kd_R12*e_omega_Eig + omega_Eig.cross(J_Eig*omega_Eig);
            
            // =========== Calculate f_thrust =========== // (I'm not sure what this does yet)
            b3_Eig = R_Eig.col(2); // current orientation of b3 vector
            f_thrust = f_total_thrust_Eig.dot(b3_Eig);

        } 
        else if (type == 3 || type==4)
        {
            if (type == 3) // attitude control
            {
                eul_Eig = control_vals_Eig;

                R_d_Eig  <<  cos(eul_Eig(1)),  0,  sin(eul_Eig(1)),
                             0,            1,  0,
                            -sin(eul_Eig(1)),  0,  cos(eul_Eig(1));

                e_R_Eig = dehat(0.5*(R_d_Eig.transpose()*R_Eig - R_Eig.transpose()*R_d_Eig));
                e_omega_Eig = omega_Eig; // This is the wrong way and purely temporary to keep consistent with the current controller =============
            }
            else if (type == 4)// Angular velocity control
            {
                omega_d_Eig = control_vals_Eig;

                omega_d_Eig(0) = omega_Eig(0) + omega_d_Eig(0); // I can't quite follow this ================
                omega_d_Eig(2) = omega_Eig(2) + omega_d_Eig(2);
                
                e_R_Eig << 0,0,0;
                e_omega_Eig = omega_Eig - omega_d_Eig;
            }

            if (R_Eig(2,2) > 0.7) // Why? It sets a cap for some reason ================
                f_thrust = f_hover/R_Eig(2,2);
            else 
                f_thrust = f_hover/0.7;

            tau_Eig = -kp_R34*e_R_Eig + -kd_R34*e_omega_Eig + omega_Eig.cross(J_Eig*omega_Eig); 
        }
        else if (type == 5)// If command[0] = 5 stop all thrust
        {
            f_thrust = 0;
            tau_Eig << 0,0,0;
            
        }
        

        FT_Eig << f_thrust, tau_Eig; // Not sure on title [f_thrust, tau(0), tau(1), tau(2)]
        f_Eig = Gamma_inv_Eig*FT_Eig; // I dont like not knowing where this comes from. 
        motorspeed_square_Eig = f_Eig/c_T; 
        
        

        // If squared motorspeed^2 is negative cap at zero
        // Why that'd be the case? I don't know
        for(int k_motor=0;k_motor<4;k_motor++)
        {
            if(motorspeed_square_Eig(k_motor)<0){
                motorspeed_square_Eig(k_motor) = 0;}
            else if (isnan(motorspeed_square_Eig(k_motor))){
               motorspeed_square_Eig(k_motor) = 0;}
        }
        
 

        if(R_Eig(2,2)<0) 
        {
            motorspeed_square_Eig(0) = 0;
            motorspeed_square_Eig(1) = 0;
            motorspeed_square_Eig(2) = 0;
            motorspeed_square_Eig(3) = 0;
        }

        motorspeed_Eig = motorspeed_square_Eig.array().sqrt();
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

    /*double result[3][3];
    double quat[4] = {9,7,3,7};
    math::quat2rotm_Rodrigue((double *) result, quat);

    cout<<"Rotation matrix is :"<<endl;           // confirm result with matlab
    cout<<result[0][0]<<", "<<result[0][1]<<", "<<result[0][2]<<endl;
    cout<<result[1][0]<<", "<<result[1][1]<<", "<<result[1][2]<<endl;
    cout<<result[2][0]<<", "<<result[2][1]<<", "<<result[2][2]<<endl;*/

    RowVector4d a(1,2,3,4);
    cout << "a: " << a << endl;
    cout << a.segment(1,3) << endl;






    while(1)
    {
        sleep(1e7);         // 1e7 is about 100 days
    }
    
    return 0;
}