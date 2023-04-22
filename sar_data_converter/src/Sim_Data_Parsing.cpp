#include "SAR_DataConverter.h"

void SAR_DataConverter::CtrlData_Callback(const crazyflie_msgs::CtrlData &ctrl_msg)
{
    // ===================
    //     FLIGHT DATA
    // ===================
    Time = ros::Time::now();
    Pose = ctrl_msg.Pose;
    Twist = ctrl_msg.Twist;
    Vel_mag = sqrt(pow(Twist.linear.x,2)+pow(Twist.linear.y,2)+pow(Twist.linear.z,2));
    Phi = atan2(Twist.linear.z,Twist.linear.x)*180/M_PI;
    Alpha = atan2(Twist.linear.y,Twist.linear.x)*180/M_PI;

    // PROCESS EULER ANGLES
    float quat[4] = {
        (float)ctrl_msg.Pose.orientation.x,
        (float)ctrl_msg.Pose.orientation.y,
        (float)ctrl_msg.Pose.orientation.z,
        (float)ctrl_msg.Pose.orientation.w
    };
    float eul[3];
    quat2euler(quat,eul);
    Eul.x = eul[0]*180/M_PI;
    Eul.y = eul[1]*180/M_PI;
    Eul.z = eul[2]*180/M_PI;

    // OPTICAL FLOW STATES
    Tau = ctrl_msg.Tau;
    Theta_x = ctrl_msg.Theta_x;
    Theta_y = ctrl_msg.Theta_y;
    D_perp = ctrl_msg.D_perp;

    // ESTIMATED OPTICAL FLOW STATES
    Tau_est = ctrl_msg.Tau_est;
    Theta_x_est = ctrl_msg.Theta_x_est;
    Theta_y_est = ctrl_msg.Theta_y_est;    

    // STATE SETPOINTS
    x_d = ctrl_msg.x_d;
    v_d = ctrl_msg.v_d;
    a_d = ctrl_msg.a_d;

    // CONTROL ACTIONS
    FM = ctrl_msg.FM;
    MotorThrusts = ctrl_msg.MotorThrusts;
    MS_PWM = ctrl_msg.MS_PWM;

    // RL POLICY DATA
    Tau_thr = ctrl_msg.Tau_thr;
    G1 = ctrl_msg.G1;

    // NEURAL NETWORK DATA
    Policy_Flip = ctrl_msg.Policy_Flip;
    Policy_Action = ctrl_msg.Policy_Action;

    Pose_impact_buff.push_back(Pose);
    Twist_impact_buff.push_back(Twist);


    // =================
    //     FLIP DATA
    // =================

    // CARTESIAN SPACE DATA
    if(ctrl_msg.flip_flag == true && OnceFlag_flip == false)
    {   
        Time_tr = ros::Time::now();
        OnceFlag_flip = true;

    }
    

    flip_flag = ctrl_msg.flip_flag;
    Pose_tr = ctrl_msg.Pose_tr;
    Twist_tr = ctrl_msg.Twist_tr;

    // PROCESS EULER ANGLES
    float quat_tr[4] = {
        (float)ctrl_msg.Pose_tr.orientation.x,
        (float)ctrl_msg.Pose_tr.orientation.y,
        (float)ctrl_msg.Pose_tr.orientation.z,
        (float)ctrl_msg.Pose_tr.orientation.w
    };
    float eul_tr[3];
    quat2euler(quat_tr,eul_tr);
    Eul_tr.x = eul_tr[0]*180/M_PI;
    Eul_tr.y = eul_tr[1]*180/M_PI;
    Eul_tr.z = eul_tr[2]*180/M_PI;

    // OPTICAL FLOW
    Tau_tr = ctrl_msg.Tau_tr;
    Theta_x_tr = ctrl_msg.Theta_x_tr;
    Theta_y_tr = ctrl_msg.Theta_y_tr;
    D_perp_tr = ctrl_msg.D_perp_tr;

    // CONTROLLER ACTIONS
    FM_tr = ctrl_msg.FM_flip;

    // NEURAL NETWORK DATA
    Policy_Flip_tr = ctrl_msg.Policy_Flip_tr;
    Policy_Action_tr = ctrl_msg.Policy_Action_tr;

}

void SAR_DataConverter::CtrlDebug_Callback(const crazyflie_msgs::CtrlDebug &ctrl_msg)
{
    Motorstop_Flag = ctrl_msg.Motorstop_Flag;
    Pos_Ctrl_Flag = ctrl_msg.Pos_Ctrl;
    Vel_Ctrl_Flag = ctrl_msg.Vel_Ctrl;
    Traj_Active_Flag = ctrl_msg.Traj_Active;
    Tumble_Detection = ctrl_msg.Tumble_Detection;
    Tumbled_Flag = ctrl_msg.Tumbled_Flag;
    Moment_Flag = ctrl_msg.Moment_Flag;
    Policy_Armed_Flag = ctrl_msg.Policy_Armed;
    Camera_Sensor_Active = ctrl_msg.Camera_Sensor_Active;
}