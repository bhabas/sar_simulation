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
        cout<<"Send initial motor speed "<<len<<" byte to Gazebo Succeeded! Avoiding threads mutual locking"<<endl;
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
        cout<<"Socket binding to RL failed"<<endl;
    else
        cout<<"Socket binding to RL succeeded"<<endl;
    
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





Vector3d vee(Matrix3d a_hat) // Input a skew-symmetric matrix and output corresponding vector
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
  // You dehat/vee a skew-symmetric matrix and get a vector

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

    // State Declarations
    double position[3];
    double orientation_q[4];
    double eul[3];
    double vel[3];
    double omega[3];

    // Rotation Matrices
    double R[3][3];
    double R_d[3][3] = {
        {1,0,0}, 
        {0,1,0}, 
        {0,0,1}};
    double e_R[3];

    
    double omega_d[3];
    double e_omega[3];

    double b1_d[3]; // b1d desired direction of body fixed axis in parametric form
    double b2_d[3];
    double b3_d[3];

    double b2_d_hat[3][3];
    double b3_d_hat[3][3];

    double v_d[3];// = {0.0, 0, 1.0};
    double e_v[3];
    
    double p_d[3];
    double e_x[3];


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

    double Gamma_inv[4][4] = {  //????
        {0.25, -7.6859225, -7.6859225, -41.914296}, 
        {0.25,  7.6859225,  7.6859225, -41.914296}, 
        {0.25,  7.6859225, -7.6859225,  41.914296},  
        {0.25, -7.6859225,  7.6859225,  41.914296}};    // calculated by Matlab

    double J[3][3] = { // Rotational Inertia
        {1.65717e-05, 0, 0}, 
        {0, 1.66556e-05, 0}, 
        {0, 0, 2.92617e-05}};


    // =====================================
    //    Array -> Matrix -> Array Example
    // =====================================

    // // cout J array
    // std::cout << "C array:\n";
    // for (int i = 0; i < 3; ++i) {
    //     for (int j = 0; j < 3; ++j) {
    //         std::cout << J[i][j] << " ";
    // }
    // std::cout << "\n";
    // }


    // // Map J array to eigen matrix
    // typedef Matrix<double, 3, 3, RowMajor> RowMatrix3d; 
    //     // - creates shortcut for matrix type: 3x3 double and RowMajor to match c++ array format
    // Map<RowMatrix3d> J_eig(&J[0][0]); // Not quite sure what &J[0][0] does but it works
    // cout << "Eigen matrix:\n" << J_eig << endl;


    // // Maps J_eig matrix to J_2 array
    // double J_2[3][3];
    // Map<RowMatrix3d> (&J_2[0][0],3,3) = J_eig;

    // std::cout << "C array2:\n";
    // for (int i = 0; i < 3; ++i) {
    //     for (int j = 0; j < 3; ++j) {
    //         std::cout << J_2[i][j] << " ";
    // }
    // std::cout << "\n";
    // }



    // =====================================
    //        Vector -> Array Example
    // =====================================
    // double a[3] = {1,2,3};
    // RowVector3d aE(1,2,3);

    // cout << "C array: " << a[0] << " " << a[1] << " " <<  a[2] << " " << a << endl;
    // cout << "Eig Vector: " << aE << endl;

    // Map<RowVector3d>(&a[0],1,3) = aE;
    // cout << "C array: " << a[0] << " " << a[1] << " " <<  a[2] << " " << a << endl;





    // double *a2; 
    // a2 = aE.data();
    // cout << "C array: " << a2[0] << " " << a2[1] << " " <<  a2[2] << " " << a << endl;



    double f_thrust =0;
    // might need to adjust weight to real case (sdf file too)
    double f_hover = (0.026 + 0.00075*4)*9.8066;
    double tau[3] =  {0,0,0};

    Map<RowVector3d> v1(tau); // uses v1 as a Vector3d object
    Map<Vector3d> v2(tau);
  


    double FT[4];
    double f[4];
    double motorspeed_square[4];

    double tmp1[3][3];  double tmp2[3][3];  double tmp3[3][3];  double tmp4[3][3];  double tmp5[3][3];  double tmp6[3][3];
    double tmp7[3]; double tmp8[3]; double tmp9[3]; double tmp10[3];    double tmp11[3][3]; double tmp12[3];    double tmp13[3];    double tmp14[3];
    double tmp21[3];    double tmp22[3];    double tmp23[3];    double tmp24[3];    double f_total_thrust[3];

    unsigned int k_run = 0;

    while(isRunning_)
    {
        k_run++;

        // Define state_full from recieved Gazebo states
        queue_states_.wait_dequeue(state_full_structure);
        memcpy(state_full, state_full_structure.data, sizeof(state_full));


        // Define control_cmd from recieved control_cmd
        if (control_cmd_recvd[0]<10) // There is a case where control_cmd_recvd becomes 11 but I'm not sure why?
            memcpy(control_cmd, control_cmd_recvd, sizeof(control_cmd)); // Rename received control_cmd for reasons


        // else if ( (control_cmd_recvd[0]>10) && (control_cmd_recvd[1]<0.5) )
        // {
        //     //cout<<"======================================="<<endl;
        //     //cout<<"Enter reset mode"<<endl;
        //     motorspeed[0] = 0.0;  
        //     motorspeed[1] = 0.0;  
        //     motorspeed[2] = 0.0;  
        //     motorspeed[3] = 0.0;
        //     sendto(fd_gazebo_, motorspeed, sizeof(motorspeed),0, (struct sockaddr*)&sockaddr_remote_gazebo_, sockaddr_remote_gazebo_len_);

        //     control_cmd_recvd[0] = 2; 
        //     control_cmd_recvd[1] = 0; 
        //     control_cmd_recvd[2] = 0; 
        //     control_cmd_recvd[3] = 0; 
        //     control_cmd_recvd[4] = 0;
        //     sleep(3);
        //     for(int k_temp=1;k_temp<5;k_temp++)
        //         queue_states_.wait_dequeue(state_full_structure);
        //     memcpy(state_full, state_full_structure.data, sizeof(state_full));
        // }


        // Seperate state values from state_full
        memcpy(position, state_full, sizeof(position));
        memcpy(orientation_q, state_full+3,  sizeof(orientation_q));
        memcpy(vel, state_full+7, sizeof(vel));
        memcpy(omega, state_full+10, sizeof(omega));

        // Redefine state values in vector format
        Map<Vector3d> pos_Eig(position);
        Map<Vector3d> att_Eig(orientation_q);
        Map<Vector3d> vel_Eig(vel);
        Map<Vector3d> omega_Eig(omega);

        

        type = control_cmd[0];
        // define control vals from 1:3 in control array and map to vector
        memcpy(control_vals, control_cmd+1, sizeof(control_vals)); 
        Map<Vector3d> control_vals_Eig(control_vals);



        // These declarations will be fixed later ====================
        Vector3d p_d_Eig; // Pose-desired
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







        math::quat2rotm_Rodrigue((double *) R, orientation_q);
        Map<RowMatrix3d> R_Eig(&R[0][0]);

        if (type == 1 || type == 2)
        {
            if (type==1) // position error calc 
            {
                p_d_Eig = control_vals_Eig; // Set desired position from thread
                e_x_Eig = pos_Eig - p_d_Eig; //  Position Error
            }
            else  // velocity error calc
            {   
                v_d_Eig = control_vals_Eig; // velocity desired
                e_v_Eig = vel_Eig - v_d_Eig; // velocity error 
            }

            e_x_Eig << 0,0,0; // Errors will need to be set to zero when not being used ============



            // =========== Calculate the Total Thrust (f) =========== // 

            Vector3d e3_Eig(0,0,1);
            Vector3d f_total_thrust_Eig; // total thrust
     
            f_total_thrust_Eig = -kp_x*e_x_Eig + -kp_v*e_v_Eig + f_hover*e3_Eig; // This is in terms of the global axes (e1,e2,e3) 
            Map<RowVector3d>(&f_total_thrust[0],1,3) = f_total_thrust_Eig; // converts eigen matrix to c++ array ===============

            // If the prescribed thrust is globally z-negative then turn thrust 
            // off so it doesn't dive bomb in a firey explosion of death (or break)
            if (f_total_thrust[2]<0) 
                f_total_thrust[2] = 0.01;





            // =========== Calculate desired body fixed axes =========== // 
            Vector3d b1_d_Eig; // b1d: desired direction of body fixed axis in parametric form
            Vector3d b2_d_Eig;
            Vector3d b3_d_Eig;

            b1_d_Eig << 1,0,0; // What is this axis lining up with???? ===============
            b3_d_Eig = f_total_thrust_Eig.normalized(); 
            b2_d_Eig = b3_d_Eig.cross(b1_d_Eig);
            b2_d_Eig.normalize();
            
            // b1_d_Eig = b2_d_Eig.cross(b3_d_Eig); // Not sure why Pan redefined this axis? =============
            // b1_d_Eig.normalize(); 

            
            Map<RowVector3d>(&b1_d[0],1,3) = b1_d_Eig; // converts eigen matrix to c++ array ===============
            Map<RowVector3d>(&b3_d[0],1,3) = b3_d_Eig; // converts eigen matrix to c++ array ===============
            Map<RowVector3d>(&b2_d[0],1,3) = b2_d_Eig; // converts eigen matrix to c++ array ===============






            // =========== Calculate Rotational Error Matrix =========== // 
            


            R_d_Eig << b1_d_Eig,b2_d_Eig,b3_d_Eig; // concatinating column vectors of desired body axes
            e_R_Eig = vee(0.5*(R_d_Eig.transpose()*R_Eig - R_Eig.transpose()*R_d_Eig));
            Map<RowVector3d>(&e_R[0],1,3) = e_R_Eig; // converts eigen matrix to c++ array ===============



            // Just fix code so omega_d = zero
            memcpy(e_omega, omega, sizeof(omega)); // This is a trick to have because omega_d set = 0 so e_omega = omega-omega_d ======================= 
            
            
            e_omega_Eig = omega_Eig; // This is wrong and purely temporary to keep consistent with the current controller =============



            // =========== Calculate Moment Vector (tau/M)=========== //



            tau_Eig = -kp_R12*e_R_Eig + -kd_R12*e_omega_Eig + omega_Eig.cross(J_Eig*omega_Eig);
            Map<RowVector3d>(&tau[0],1,3) = tau_Eig; // converts eigen matrix to c++ array ===============



            // =========== Calculate f_thrust =========== // (I'm not sure what this does yet)
            Vector3d b3_Eig;
            b3_Eig = R_Eig.col(2); // current orientation of b3 vector
            f_thrust = f_total_thrust_Eig.dot(b3_Eig);


        } // Pick up here ***************
        else if (type == 3 || type==4)
        {

            Vector3d eul_Eig;

            if (type == 3) // attitude control
            {
                

                eul_Eig = control_vals_Eig;

                R_d_Eig  <<  cos(eul[1]),  0,  sin(eul[1]),
                             0,            1,  0,
                            -sin(eul[1]),  0,  cos(eul[1]);

                e_R_Eig = vee(0.5*(R_d_Eig.transpose()*R_Eig - R_Eig.transpose()*R_d_Eig));
                e_omega_Eig = omega_Eig; // This is wrong and purely temporary to keep consistent with the current controller =============

                Map<RowVector3d>(&e_R[0],1,3) = e_R_Eig; // converts eigen matrix to c++ array ===============

                memcpy(e_omega, omega, sizeof(omega));
            }
            else // Angular velocity control
            {
                omega_d_Eig = control_vals_Eig;

                omega_d_Eig(0) = omega_Eig(0) + omega_d_Eig(0); // I can't quite follow this ================
                omega_d_Eig(2) = omega_Eig(2) + omega_d_Eig(2);
                
                e_R_Eig << 0,0,0;
                Map<RowVector3d>(&e_R[0],1,3) = e_R_Eig; // converts eigen matrix to c++ array ===============

                e_omega_Eig = omega_Eig - omega_d_Eig;
                Map<RowVector3d>(&e_omega[0],1,3) = e_omega_Eig; // converts eigen matrix to c++ array ===============


            }

            if (R_Eig(2,2) > 0.7) // Why? It sets a cap for some reason ================
                f_thrust = f_hover/R_Eig(2,2);
            else 
                f_thrust = f_hover/0.7;



            tau_Eig = -kp_R34*e_R_Eig + -kd_R34*e_omega_Eig + omega_Eig.cross(J_Eig*omega_Eig); 
            Map<RowVector3d>(&tau[0],1,3) = tau_Eig; // converts eigen matrix to c++ array ===============


        }
        else // If command[0] = 5 stop all thrust
        {
            f_thrust = 0;
            tau_Eig << 0,0,0;
            Map<RowVector3d>(&tau[0],1,3) = tau_Eig; // converts eigen matrix to c++ array ===============

        }
        


        

        

        FT[0] = f_thrust;//f_thrust;
        FT[1] = tau[0];
        FT[2] = tau[1];
        FT[3] = tau[2];
        math::matTimesVec(f, (double *) Gamma_inv, FT, 4);
        math::matTimesScalar(motorspeed_square, f, c_T, 4, 2);
        //cout << motorspeed_square[0] << endl;
        for(int k_ms_s=0;k_ms_s<4;k_ms_s++)
        {
            if(motorspeed_square[k_ms_s]<0) {
                motorspeed_square[k_ms_s] = 0;}
            else if (isnan(motorspeed_square[k_ms_s])) {
                motorspeed_square[k_ms_s] = 0; }

        }
        if(type == 3 || type == 4)
        {
            motorspeed_square[0]= (motorspeed_square[0]+motorspeed_square[2])/2;     
            motorspeed_square[2]=motorspeed_square[0];


            if(R[2][2]<0)
            {
                /*if (k_run%100 == 1)
                    cout<<"Shutdown motors"<<endl;*/
                motorspeed_square[0] = 0;   
                motorspeed_square[1] = 0;   
                motorspeed_square[2] = 0;   
                motorspeed_square[3] = 0;
            }
            
        }

        
        motorspeed[0] = sqrt(motorspeed_square[0]);
        motorspeed[1] = sqrt(motorspeed_square[1]);
        motorspeed[2] = sqrt(motorspeed_square[2]);
        motorspeed[3] = sqrt(motorspeed_square[3]);

        double ms_min = 0.0;
        double ms_max = 3420.0;
        for (int i =0; i<4;i++) { // clamp motor speed (based on max thrust (0.6) speed)
            motorspeed[i] = math::clamp(motorspeed[i],ms_min,ms_max);
        }
        // cout causing wierd behavior?????
        //if ( k_run%50 == 1 ) {
        //        cout<<"motor speed ["<< motorspeed[0]<<", "<< motorspeed[1]<<", "<< motorspeed[2]<<", "<<motorspeed[3]<<"]"<<endl;
        //}   
        //memcpy(motorspeed_structure.data, motorspeed, sizeof(motorspeed));
        //queue_motorspeed_.enqueue(motorspeed_structure);
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


    Matrix2d a;
    a << 1,2,3,4;
    Vector3d b;
    b << 1,2,3;

 




    while(1)
    {
        sleep(1e7);         // 1e7 is about 100 days
    }
    
    return 0;
}