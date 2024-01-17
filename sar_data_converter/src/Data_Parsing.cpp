#include "SAR_DataConverter.h"

void SAR_DataConverter::CtrlData_Callback(const sar_msgs::CTRL_Data &ctrl_msg)
{
    // ===================
    //     FLIGHT DATA
    // ===================
    Time_prev = Time;
    Time = ros::Time::now();

    Pose = ctrl_msg.Pose;
    Twist = ctrl_msg.Twist;
    Accel = ctrl_msg.Accel;
    Acc_mag = ctrl_msg.AccMag;
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

    // STATES RELATIVE TO LANDING SURFACE
    D_perp = ctrl_msg.D_perp;
    V_perp = ctrl_msg.V_perp;
    V_tx = ctrl_msg.V_tx;
    V_ty = ctrl_msg.V_ty;

    // OPTICAL FLOW STATES
    Tau = ctrl_msg.Tau;
    Theta_x = ctrl_msg.Theta_x;
    Theta_y = ctrl_msg.Theta_y;

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


    // NEURAL NETWORK DATA
    Policy_Trg_Action = ctrl_msg.Policy_Trg_Action;
    Policy_Flip_Action = ctrl_msg.Policy_Flip_Action;

    Pose_impact_buff.push_back(Pose);
    Twist_impact_buff.push_back(Twist);
    Accel_impact_buff.push_back(Accel);


    // =================
    //     FLIP DATA
    // =================

    // CARTESIAN SPACE DATA
    if(ctrl_msg.flip_flag == true && OnceFlag_flip == false)
    {   
        Time_trg = ros::Time::now();
        OnceFlag_flip = true;

    }

    if(ctrl_msg.flip_flag == true && impact_flag == false)
    {
        double Time_delta = Time.toSec()-Time_prev.toSec();
        Rot_Sum += (Time_delta*Twist.angular.y)*180/M_PI;
        // printf("Val: %f\n",Rot_Sum);
    }
    

    flip_flag = ctrl_msg.flip_flag;
    Pose_trg = ctrl_msg.Pose_trg;
    Twist_trg = ctrl_msg.Twist_trg;
    Accel_trg = ctrl_msg.Accel_trg;

    // PROCESS EULER ANGLES
    float quat_trg[4] = {
        (float)ctrl_msg.Pose_trg.orientation.x,
        (float)ctrl_msg.Pose_trg.orientation.y,
        (float)ctrl_msg.Pose_trg.orientation.z,
        (float)ctrl_msg.Pose_trg.orientation.w
    };
    float eul_trg[3];
    quat2euler(quat_trg,eul_trg);
    Eul_trg.x = eul_trg[0]*180/M_PI;
    Eul_trg.y = eul_trg[1]*180/M_PI;
    Eul_trg.z = eul_trg[2]*180/M_PI;

    // OPTICAL FLOW
    Tau_trg = ctrl_msg.Tau_trg;
    Theta_x_trg = ctrl_msg.Theta_x_trg;
    Theta_y_trg = ctrl_msg.Theta_y_trg;
    D_perp_trg = ctrl_msg.D_perp_trg;

    // CONTROLLER ACTIONS
    FM_trg = ctrl_msg.FM_flip;

    // NEURAL NETWORK DATA
    Policy_Trg_Action_trg = ctrl_msg.Policy_Trg_Action_trg;
    Policy_Flip_Action_trg = ctrl_msg.Policy_Flip_Action_trg;

}

void SAR_DataConverter::CtrlDebug_Callback(const sar_msgs::CTRL_Debug &ctrl_msg)
{
    Motorstop_Flag = ctrl_msg.Motorstop_Flag;
    Pos_Ctrl_Flag = ctrl_msg.Pos_Ctrl;
    Vel_Ctrl_Flag = ctrl_msg.Vel_Ctrl;
    Traj_Active_Flag = ctrl_msg.Traj_Active;
    Tumble_Detection = ctrl_msg.Tumble_Detection;
    Tumbled_Flag = ctrl_msg.Tumbled_Flag;
    Moment_Flag = ctrl_msg.Moment_Flag;
    Policy_Armed_Flag = ctrl_msg.Policy_Armed;
}



void SAR_DataConverter::decompressXY(uint32_t xy, float xy_arr[])
{
    uint16_t xd = ((uint32_t)xy >> 16) & 0xFFFF;    // Shift out y-value bits
    xy_arr[0] = ((float)xd - 32767.0f)*1e-3;        // Convert to normal value

    uint16_t yd = (uint32_t)xy & 0xFFFF;            // Save only y-value bits
    xy_arr[1] = ((float)yd - 32767.0f)*1e-3;

}



void SAR_DataConverter::cf1_FullState_Callback(const sar_msgs::GenericLogData::ConstPtr &log_msg)
{
    // ===================
    //     FLIGHT DATA
    // ===================
    Time = ros::Time::now();

    // POSITION
    float xy_arr[2];
    decompressXY(log_msg->values[0],xy_arr);

    Pose.position.x = xy_arr[0];
    Pose.position.y = xy_arr[1];
    Pose.position.z = log_msg->values[1]*1e-3;

    // VELOCITY
    float vxy_arr[2];
    decompressXY(log_msg->values[2],vxy_arr);
    
    Twist.linear.x = vxy_arr[0];
    Twist.linear.y = vxy_arr[1];
    Twist.linear.z = log_msg->values[3]*1e-3;
    Vel_mag = sqrt(pow(Twist.linear.x,2)+pow(Twist.linear.y,2)+pow(Twist.linear.z,2));
    Phi = atan2(Twist.linear.z,Twist.linear.x)*180/M_PI;
    Alpha = atan2(Twist.linear.y,Twist.linear.x)*180/M_PI;

    // ORIENTATION
    float quat[4];
    uint32_t quatZ = (uint32_t)log_msg->values[4];
    quatdecompress(quatZ,quat);

    Pose.orientation.x = quat[0];
    Pose.orientation.y = quat[1];
    Pose.orientation.z = quat[2];
    Pose.orientation.w = quat[3]; 

    // PROCESS EULER ANGLES
    float eul[3];
    quat2euler(quat,eul);
    Eul.x = eul[0]*180/M_PI;
    Eul.y = eul[1]*180/M_PI;
    Eul.z = eul[2]*180/M_PI;

    // ANGULAR VELOCITY
    float wxy_arr[2];
    decompressXY(log_msg->values[5],wxy_arr);
    
    Twist.angular.x = wxy_arr[0]*10;
    Twist.angular.y = wxy_arr[1]*10;
    Twist.angular.z = log_msg->values[6]*1e-3;

}

void SAR_DataConverter::cf1_PolicyState_Callback(const sar_msgs::GenericLogData::ConstPtr &log_msg)
{
    // CEILING DISTANCE
    D_perp = log_msg->values[0]*1e-3;

    // OPTICAL FLOW VALUES
    float Theta_xy_arr[2];
    decompressXY(log_msg->values[1],Theta_xy_arr);
    
    Theta_x = Theta_xy_arr[0];
    Theta_y = Theta_xy_arr[1];
    Tau = log_msg->values[2]*1e-3;


    // OPTICAL FLOW ESTIMATES
    float Theta_xy_est_arr[2];
    decompressXY(log_msg->values[3],Theta_xy_est_arr);
    
    Theta_x_est = Theta_xy_est_arr[0];
    Theta_y_est = Theta_xy_est_arr[1];
    Tau_est = log_msg->values[4]*1e-3;


    // POLICY ACTIONS
    float Policy_Action_arr[2];
    decompressXY(log_msg->values[5],Policy_Action_arr);
    Policy_Trg_Action = Policy_Action_arr[0];
    Policy_Flip_Action = Policy_Action_arr[1];

    // FLIP FLAG
    flip_flag = log_msg->values[6];
    if(flip_flag == true && OnceFlag_flip == false)
    {
        Time_trg = ros::Time::now();
        OnceFlag_flip = true;
    }

}

void SAR_DataConverter::cf1_CTRL_Output_Callback(const sar_msgs::GenericLogData::ConstPtr &log_msg)
{
    // DECOMPRESS THRUST/MOMENT MOTOR VALUES [g]
    float M_xy[2];
    float FM_z[2];

    decompressXY(log_msg->values[0],M_xy); 
    decompressXY(log_msg->values[1],FM_z);

    FM = {FM_z[0],M_xy[0],M_xy[1],FM_z[1]}; // [F,Mx,My,Mz]

    // MOTOR THRUST VALUES
    float M_thrust12[2];
    float M_thrust34[2];

    decompressXY(log_msg->values[2],M_thrust12);
    decompressXY(log_msg->values[3],M_thrust34);

    MotorThrusts = {M_thrust12[0],M_thrust12[1],M_thrust34[0],M_thrust34[1]};

    // MOTOR PWM VALUES
    float MS_PWM12[2];
    float MS_PWM34[2];

    decompressXY(log_msg->values[4],MS_PWM12);
    decompressXY(log_msg->values[5],MS_PWM34);

    MS_PWM = {
        (uint16_t)round(MS_PWM12[0]*2.0e3),
        (uint16_t)round(MS_PWM12[1]*2.0e3), 
        (uint16_t)round(MS_PWM34[0]*2.0e3),
        (uint16_t)round(MS_PWM34[1]*2.0e3)
    };
    
}

void SAR_DataConverter::cf1_SetPoints_Callback(const sar_msgs::GenericLogData::ConstPtr &log_msg)
{
    // POSITION SETPOINTS
    float xd_xy[2];
    decompressXY(log_msg->values[0],xd_xy);

    x_d.x = xd_xy[0];
    x_d.y = xd_xy[1];
    x_d.z = log_msg->values[1]*1e-3;
   
    // VELOCITY SETPOINTS
    float vd_xy[2];
    decompressXY(log_msg->values[2],vd_xy);

    v_d.x = vd_xy[0];
    v_d.y = vd_xy[1];
    v_d.z = log_msg->values[3]*1e-3;

    // ACCELERATION SETPOINTS
    float ad_xy[2];
    decompressXY(log_msg->values[4],ad_xy);

    a_d.x = ad_xy[0];
    a_d.y = ad_xy[1];
    a_d.z = log_msg->values[5]*1e-3;

}

void SAR_DataConverter::cf1_TrgState_Callback(const sar_msgs::GenericLogData::ConstPtr &log_msg)
{
    // TRIGGER STATE - CEILING DISTANCE
    D_perp_trg = log_msg->values[0]*1e-3;

    // TRIGGER STATE - POSITION
    Pose_trg.position.x = NAN;
    Pose_trg.position.y = NAN;
    Pose_trg.position.z = log_msg->values[1]*1e-3;


    // TRIGGER STATE - VELOCITY
    float vxy_arr[2];
    decompressXY(log_msg->values[2],vxy_arr);
    
    Twist_trg.linear.x = vxy_arr[0];
    Twist_trg.linear.y = vxy_arr[1];
    Twist_trg.linear.z = log_msg->values[3]*1e-3;

    // TRIGGER STATE - OPTICAL FLOW
    float Theta_xy_arr[2];
    decompressXY(log_msg->values[4],Theta_xy_arr);
    
    Theta_x_trg = Theta_xy_arr[0];
    Theta_y_trg = Theta_xy_arr[1];
    Tau_trg = log_msg->values[5]*1e-3;

    // TRIGGER STATE - OPTICAL FLOW ESTIMATE
    float Theta_xy_est_arr[2];
    decompressXY(log_msg->values[6],Theta_xy_est_arr);
    
    Theta_x_trg = Theta_xy_est_arr[0];
    Theta_y_trg = Theta_xy_est_arr[1];
    Tau_trg = log_msg->values[7]*1e-3;

    // TRIGGER STATE - POLICY ACTIONS
    float Policy_Action_arr[2];
    decompressXY(log_msg->values[8],Policy_Action_arr);
    Policy_Trg_Action_trg = Policy_Action_arr[0];
    Policy_Flip_Action_trg = Policy_Action_arr[1];
    
    
}

void SAR_DataConverter::cf1_Flags_Callback(const sar_msgs::GenericLogData::ConstPtr &log_msg)
{

    Pos_Ctrl_Flag = (bool)log_msg->values[0];
    Vel_Ctrl_Flag = (bool)log_msg->values[1];
    Motorstop_Flag = (bool)log_msg->values[2];
    Moment_Flag = (bool)log_msg->values[3];
    Tumbled_Flag = (bool)log_msg->values[4];
    Tumble_Detection = (bool)log_msg->values[5];
    Policy_Armed_Flag = (bool)log_msg->values[6];
    isCamActive = (bool)log_msg->values[7];

    // V_battery = log_msg->values[0];
}


void SAR_DataConverter::cf1_Misc_Callback(const sar_msgs::GenericLogData::ConstPtr &log_msg)
{
    CustomThrust_Flag = (bool)log_msg->values[0];
    CustomPWM_Flag = (bool)log_msg->values[1];
    SafeModeEnable = (bool)log_msg->values[2];
    AttCtrl_Flag = (bool)log_msg->values[3];

}
