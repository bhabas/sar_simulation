#include <iostream>

#include <Eigen/Dense>
#include "controller.h"

#include <ros/ros.h>



void Controller::controlThread()
{

    

    

    // XY POSITION PID
    float P_kp_xy = 0.5f;
    float P_kd_xy = 0.3f;
    float P_ki_xy = 0.1f;
    float i_range_xy = 0.3f;

    // Z POSITION PID
    float P_kp_z = 1.2f;
    float P_kd_z = 0.35f;
    float P_ki_z = 0.1f;
    float i_range_z = 0.25f;

    // XY ATTITUDE PID
    float R_kp_xy = 0.004f;
    float R_kd_xy = 0.0017f;
    float R_ki_xy = 0.0f;
    float i_range_R_xy = 1.0f;

    // Z ATTITUDE PID
    float R_kp_z = 0.003f;
    float R_kd_z = 0.001f;
    float R_ki_z = 0.002;
    float i_range_R_z = 0.5f;

    int a = 1;
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

        cout << Kp_p.x << endl;

        // =========== STATE DEFINITIONS =========== //
        statePos = mkvec(0.0f,0.0f,0.0f);                      // [m]
        stateVel = mkvec(0.0f,0.0f,0.0f);                      // [m]
        stateOmega = mkvec(0.0f,0.0f,0.0f);   // [rad/s]
        stateQuat = mkquat(0.0f,0.0f,0.0f,1.0f);

        RREV = stateVel.z/(h_ceiling - statePos.z);
        OF_x = stateVel.y/(h_ceiling - statePos.z);
        OF_y = stateVel.x/(h_ceiling - statePos.z);



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