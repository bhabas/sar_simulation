#include "SAR_DataConverter.h"

void SAR_DataConverter::CtrlData_Callback(const sar_msgs::CTRL_Data &ctrl_msg)
{
    // ===================
    //     FLIGHT DATA
    // ===================
    Time_prev = Time;
    Time = ros::Time::now();


    // STATES WRT ORIGIN
    Pose_B_O = ctrl_msg.Pose_B_O;
    Pose_B_O.orientation.x = NAN; // Quaternion is not used
    Pose_B_O.orientation.y = NAN;
    Pose_B_O.orientation.z = NAN;
    Pose_B_O.orientation.w = NAN;
    Twist_B_O = ctrl_msg.Twist_B_O;
    Accel_B_O = ctrl_msg.Accel_B_O;
    Accel_B_O_Mag = ctrl_msg.Accel_B_O_Mag;

    // PROCESS EULER ANGLES
    float quat[4] = {
        (float)ctrl_msg.Pose_B_O.orientation.x,
        (float)ctrl_msg.Pose_B_O.orientation.y,
        (float)ctrl_msg.Pose_B_O.orientation.z,
        (float)ctrl_msg.Pose_B_O.orientation.w
    };
    float eul[3];
    quat2euler(quat,eul);
    Eul_B_O.x = eul[0]*180/M_PI;
    Eul_B_O.y = eul[1]*180/M_PI;
    Eul_B_O.z = eul[2]*180/M_PI;

    Vel_mag_B_O = sqrt(pow(Twist_B_O.linear.x,2)+pow(Twist_B_O.linear.z,2));
    Vel_angle_B_O = atan2(Twist_B_O.linear.z,Twist_B_O.linear.x)*180/M_PI;


    // STATES WRT PLANE
    Pose_P_B = ctrl_msg.Pose_P_B;
    Pose_P_B.orientation.x = NAN; // Quaternion is not used
    Pose_P_B.orientation.y = NAN;
    Pose_P_B.orientation.z = NAN;
    Pose_P_B.orientation.w = NAN;

    Twist_B_P = ctrl_msg.Twist_B_P;
    Vel_mag_B_P = ctrl_msg.Vel_mag_B_P;
    Vel_angle_B_P = ctrl_msg.Vel_angle_B_P;

    Eul_P_B.x = NAN;
    Eul_P_B.y = Plane_Angle_deg - Eul_B_O.y;
    Eul_P_B.z = NAN;
    

    // STATES RELATIVE TO LANDING SURFACE
    D_perp = ctrl_msg.D_perp;
    D_perp_CR = ctrl_msg.D_perp_CR;

    if (D_perp < D_perp_min)
    {
        D_perp_min = D_perp;
    }

    // OPTICAL FLOW STATES
    Optical_Flow = ctrl_msg.Optical_Flow;
    Optical_Flow_Cam = ctrl_msg.Optical_Flow_Cam;
    
    Theta_x = Optical_Flow.x;
    Theta_y = Optical_Flow.y;
    Tau = Optical_Flow.z;
    Tau_CR = ctrl_msg.Tau_CR;

    // ESTIMATED OPTICAL FLOW STATES
    Theta_x_Cam = Optical_Flow_Cam.x;
    Theta_y_Cam = Optical_Flow_Cam.y;    
    Tau_Cam = Optical_Flow_Cam.z;

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
    Policy_Rot_Action = ctrl_msg.Policy_Rot_Action;

    Pose_B_O_impact_buff.push_back(Pose_B_O);
    Eul_B_O_impact_buff.push_back(Eul_B_O);

    Twist_P_B_impact_buff.push_back(Twist_B_P);
    Eul_P_B_impact_buff.push_back(Eul_P_B);


    // =================
    //   TRIGGER DATA
    // =================

    Trg_Flag = ctrl_msg.Trg_Flag;
    if(ctrl_msg.Trg_Flag == true && OnceFlag_Trg == false)
    {   
        Time_trg = ros::Time::now();
        OnceFlag_Trg = true;
        Rot_Sum = Eul_B_O.y;

    }

    if(ctrl_msg.Trg_Flag == true)
    {
        double Time_delta = Time.toSec()-Time_prev.toSec();
        Rot_Sum += (Time_delta*Twist_B_O.angular.y)*180/M_PI;
        // printf("Val: %f\n",Rot_Sum);
    }
    

    
    // STATES WRT ORIGIN
    Pose_B_O_trg = ctrl_msg.Pose_B_O_trg;
    Pose_B_O_trg.orientation.x = NAN; // Quaternion is not used
    Pose_B_O_trg.orientation.y = NAN;
    Pose_B_O_trg.orientation.z = NAN;
    Pose_B_O_trg.orientation.w = NAN;
    Twist_B_O_trg = ctrl_msg.Twist_B_O_trg;

    Vel_mag_B_O_trg = sqrt(pow(Twist_B_O_trg.linear.x,2)+pow(Twist_B_O_trg.linear.z,2));
    Vel_angle_B_O_trg = atan2(Twist_B_O_trg.linear.z,Twist_B_O_trg.linear.x)*180/M_PI;

    float quat_trg[4] = {
        (float)ctrl_msg.Pose_B_O_trg.orientation.x,
        (float)ctrl_msg.Pose_B_O_trg.orientation.y,
        (float)ctrl_msg.Pose_B_O_trg.orientation.z,
        (float)ctrl_msg.Pose_B_O_trg.orientation.w
    };
    float eul_trg[3];
    quat2euler(quat_trg,eul_trg);
    Eul_B_O_trg.x = eul_trg[0]*180/M_PI;
    Eul_B_O_trg.y = eul_trg[1]*180/M_PI;
    Eul_B_O_trg.z = eul_trg[2]*180/M_PI;

    // STATES WRT PLANE
    Pose_P_B_trg = ctrl_msg.Pose_P_B_trg;
    Pose_P_B_trg.orientation.x = NAN; // Quaternion is not used
    Pose_P_B_trg.orientation.y = NAN;
    Pose_P_B_trg.orientation.z = NAN;
    Pose_P_B_trg.orientation.w = NAN;
    Twist_B_P_trg = ctrl_msg.Twist_B_P_trg;

    Eul_P_B_trg.x = NAN;
    Eul_P_B_trg.y = Plane_Angle_deg - Eul_B_O_trg.y;
    Eul_P_B_trg.z = NAN;

    Vel_mag_B_P_trg = ctrl_msg.Vel_mag_B_P_trg;
    Vel_angle_B_P_trg = ctrl_msg.Vel_angle_B_P_trg;


    // OPTICAL FLOW
    Optical_Flow_trg = ctrl_msg.Optical_Flow_trg;
    Theta_x_trg = Optical_Flow_trg.x;
    Theta_y_trg = Optical_Flow_trg.y;
    Tau_trg = Optical_Flow_trg.z;
    Tau_CR_trg = ctrl_msg.Tau_CR_trg;
    D_perp_trg = ctrl_msg.Pose_P_B_trg.position.z;
    D_perp_CR_trg = ctrl_msg.D_perp_CR_trg;


    // POLICY ACTION DATA
    Policy_Trg_Action_trg = ctrl_msg.Policy_Trg_Action_trg;
    Policy_Rot_Action_trg = ctrl_msg.Policy_Rot_Action_trg;

    // =======================
    //   ONBOARD IMPACT DATA
    // =======================

    Impact_Flag_OB = ctrl_msg.Impact_Flag_OB;

    if(ctrl_msg.Impact_Flag_OB == true && OnceFlag_Impact_OB == false)
    {   
        Time_impact_OB = ros::Time::now();
        OnceFlag_Impact_OB = true;

    }

    Pose_B_O_impact_OB = ctrl_msg.Pose_B_O_impact_OB;
    Pose_B_O_impact_OB.orientation.x = NAN; // Quaternion is not used
    Pose_B_O_impact_OB.orientation.y = NAN;
    Pose_B_O_impact_OB.orientation.z = NAN;
    Pose_B_O_impact_OB.orientation.w = NAN;
    Twist_B_P_impact_OB = ctrl_msg.Twist_B_P_impact_OB;
    Accel_B_O_Mag_impact_OB = ctrl_msg.Accel_B_O_Mag_impact_OB;

}

void SAR_DataConverter::CtrlDebug_Callback(const sar_msgs::CTRL_Debug &ctrl_msg)
{
    Tumbled_Flag = ctrl_msg.Tumbled_Flag;
    TumbleDetect_Flag = ctrl_msg.TumbleDetect_Flag;
    MotorStop_Flag = ctrl_msg.MotorStop_Flag;
    AngAccel_Flag = ctrl_msg.AngAccel_Flag;
    SafeMode_Flag = ctrl_msg.SafeMode_Flag;
    CustomThrust_Flag = ctrl_msg.CustomThrust_Flag;
    CustomPWM_Flag = ctrl_msg.CustomPWM_Flag;
    
    Pos_Ctrl_Flag = ctrl_msg.Pos_Ctrl_Flag;
    Vel_Ctrl_Flag = ctrl_msg.Vel_Ctrl_Flag;
    Policy_Armed_Flag = ctrl_msg.Policy_Armed_Flag;
    CamActive_Flag = ctrl_msg.CamActive_Flag;
}



void SAR_DataConverter::decompressXY(uint32_t xy, float xy_arr[])
{
    uint16_t xd = ((uint32_t)xy >> 16) & 0xFFFF;    // Shift out y-value bits
    xy_arr[0] = ((float)xd - 32767.0f)*1e-3;        // Convert to normal value

    uint16_t yd = (uint32_t)xy & 0xFFFF;            // Save only y-value bits
    xy_arr[1] = ((float)yd - 32767.0f)*1e-3;

}



void SAR_DataConverter::cf1_States_B_O_Callback(const sar_msgs::GenericLogData::ConstPtr &log_msg)
{
    // ===================
    //     FLIGHT DATA
    // ===================
    Time = ros::Time::now();

    // POSITION
    float xy_arr[2];
    decompressXY(log_msg->values[0],xy_arr);

    Pose_B_O.position.x = xy_arr[0];
    Pose_B_O.position.y = xy_arr[1];
    Pose_B_O.position.z = log_msg->values[1]*1e-3;


    // VELOCITY
    float vxy_arr[2];
    decompressXY(log_msg->values[2],vxy_arr);
    
    Twist_B_O.linear.x = vxy_arr[0];
    Twist_B_O.linear.y = vxy_arr[1];
    Twist_B_O.linear.z = log_msg->values[3]*1e-3;
    
    Vel_mag_B_O = sqrt(pow(Twist_B_O.linear.x,2)+pow(Twist_B_O.linear.z,2));
    Vel_angle_B_O = atan2(Twist_B_O.linear.z,Twist_B_O.linear.x)*180/M_PI;

    // ORIENTATION
    float quat[4];
    uint32_t quatZ = (uint32_t)log_msg->values[4];
    quatdecompress(quatZ,quat);

    Pose_B_O.orientation.x = quat[0];
    Pose_B_O.orientation.y = quat[1];
    Pose_B_O.orientation.z = quat[2];
    Pose_B_O.orientation.w = quat[3]; 

    // PROCESS EULER ANGLES
    float eul[3];
    quat2euler(quat,eul);
    Eul_B_O.x = eul[0]*180/M_PI;
    Eul_B_O.y = eul[1]*180/M_PI;
    Eul_B_O.z = eul[2]*180/M_PI;

    // ANGULAR VELOCITY
    float omega_xy_arr[2];
    decompressXY(log_msg->values[5],omega_xy_arr);
    
    Twist_B_O.angular.x = omega_xy_arr[0]*10;
    Twist_B_O.angular.y = omega_xy_arr[1]*10;
    Twist_B_O.angular.z = log_msg->values[6]*1e-3;

    // ACCELERATION
    Accel_B_O.angular.y = log_msg->values[7]*1e-3;


}

void SAR_DataConverter::cf1_States_B_P_Callback(const sar_msgs::GenericLogData::ConstPtr &log_msg)
{
    // // CEILING DISTANCE
    // D_perp = log_msg->values[0]*1e-3;

    // // OPTICAL FLOW VALUES
    // float Theta_xy_arr[2];
    // decompressXY(log_msg->values[1],Theta_xy_arr);
    
    // Theta_x = Theta_xy_arr[0];
    // Theta_y = Theta_xy_arr[1];
    // Tau = log_msg->values[2]*1e-3;


    // // OPTICAL FLOW ESTIMATES
    // float Theta_xy_est_arr[2];
    // decompressXY(log_msg->values[3],Theta_xy_est_arr);
    
    // Theta_x_Cam = Theta_xy_est_arr[0];
    // Theta_y_Cam = Theta_xy_est_arr[1];
    // Tau_Cam = log_msg->values[4]*1e-3;


    // // POLICY ACTIONS
    // float Policy_Action_arr[2];
    // decompressXY(log_msg->values[5],Policy_Action_arr);
    // Policy_Trg_Action = Policy_Action_arr[0];
    // Policy_Rot_Action = Policy_Action_arr[1];

    // // TRIGGER FLAG
    // Trg_Flag = log_msg->values[6];
    // if(Trg_Flag == true && OnceFlag_Trg == false)
    // {
    //     Time_trg = ros::Time::now();
    //     OnceFlag_Trg = true;
    // }

}

void SAR_DataConverter::cf1_CTRL_Output_Callback(const sar_msgs::GenericLogData::ConstPtr &log_msg)
{
    // // DECOMPRESS THRUST/MOMENT MOTOR VALUES [g]
    // float M_xy[2];
    // float FM_z[2];

    // decompressXY(log_msg->values[0],M_xy); 
    // decompressXY(log_msg->values[1],FM_z);

    // FM = {FM_z[0],M_xy[0],M_xy[1],FM_z[1]}; // [F,Mx,My,Mz]

    // // MOTOR THRUST VALUES
    // float M_thrust12[2];
    // float M_thrust34[2];

    // decompressXY(log_msg->values[2],M_thrust12);
    // decompressXY(log_msg->values[3],M_thrust34);

    // MotorThrusts = {M_thrust12[0],M_thrust12[1],M_thrust34[0],M_thrust34[1]};

    // // MOTOR PWM VALUES
    // float MS_PWM12[2];
    // float MS_PWM34[2];

    // decompressXY(log_msg->values[4],MS_PWM12);
    // decompressXY(log_msg->values[5],MS_PWM34);

    // MS_PWM = {
    //     (uint16_t)round(MS_PWM12[0]*2.0e3),
    //     (uint16_t)round(MS_PWM12[1]*2.0e3), 
    //     (uint16_t)round(MS_PWM34[0]*2.0e3),
    //     (uint16_t)round(MS_PWM34[1]*2.0e3)
    // };
    
}

void SAR_DataConverter::cf1_SetPoints_Callback(const sar_msgs::GenericLogData::ConstPtr &log_msg)
{
    // // POSITION SETPOINTS
    // float xd_xy[2];
    // decompressXY(log_msg->values[0],xd_xy);

    // x_d.x = xd_xy[0];
    // x_d.y = xd_xy[1];
    // x_d.z = log_msg->values[1]*1e-3;
   
    // // VELOCITY SETPOINTS
    // float vd_xy[2];
    // decompressXY(log_msg->values[2],vd_xy);

    // v_d.x = vd_xy[0];
    // v_d.y = vd_xy[1];
    // v_d.z = log_msg->values[3]*1e-3;

    // // ACCELERATION SETPOINTS
    // float ad_xy[2];
    // decompressXY(log_msg->values[4],ad_xy);

    // a_d.x = ad_xy[0];
    // a_d.y = ad_xy[1];
    // a_d.z = log_msg->values[5]*1e-3;

}

void SAR_DataConverter::cf1_TrgState_Callback(const sar_msgs::GenericLogData::ConstPtr &log_msg)
{
    // // TRIGGER STATE - CEILING DISTANCE
    // D_perp_trg = log_msg->values[0]*1e-3;

    // // TRIGGER STATE - POSITION
    // Pose_trg.position.x = NAN;
    // Pose_trg.position.y = NAN;
    // Pose_trg.position.z = log_msg->values[1]*1e-3;


    // // TRIGGER STATE - VELOCITY
    // float vxy_arr[2];
    // decompressXY(log_msg->values[2],vxy_arr);
    
    // Twist_trg.linear.x = vxy_arr[0];
    // Twist_trg.linear.y = vxy_arr[1];
    // Twist_trg.linear.z = log_msg->values[3]*1e-3;

    // // TRIGGER STATE - OPTICAL FLOW
    // float Theta_xy_arr[2];
    // decompressXY(log_msg->values[4],Theta_xy_arr);
    
    // Theta_x_trg = Theta_xy_arr[0];
    // Theta_y_trg = Theta_xy_arr[1];
    // Tau_trg = log_msg->values[5]*1e-3;

    // // TRIGGER STATE - OPTICAL FLOW ESTIMATE
    // float Theta_xy_est_arr[2];
    // decompressXY(log_msg->values[6],Theta_xy_est_arr);
    
    // Theta_x_trg = Theta_xy_est_arr[0];
    // Theta_y_trg = Theta_xy_est_arr[1];
    // Tau_trg = log_msg->values[7]*1e-3;

    // // TRIGGER STATE - POLICY ACTIONS
    // float Policy_Action_arr[2];
    // decompressXY(log_msg->values[8],Policy_Action_arr);
    // Policy_Trg_Action_trg = Policy_Action_arr[0];
    // Policy_Rot_Action_trg = Policy_Action_arr[1];
    
    
}

void SAR_DataConverter::cf1_Flags_Callback(const sar_msgs::GenericLogData::ConstPtr &log_msg)
{

    // Pos_Ctrl_Flag = (bool)log_msg->values[0];
    // Vel_Ctrl_Flag = (bool)log_msg->values[1];
    // MotorStop_Flag = (bool)log_msg->values[2];
    // AngAccel_Flag = (bool)log_msg->values[3];
    // Tumbled_Flag = (bool)log_msg->values[4];
    // TumbleDetect_Flag = (bool)log_msg->values[5];
    // Policy_Armed_Flag = (bool)log_msg->values[6];
    // CamActive_Flag = (bool)log_msg->values[7];

    // // V_battery = log_msg->values[0];
}


void SAR_DataConverter::cf1_Misc_Callback(const sar_msgs::GenericLogData::ConstPtr &log_msg)
{
    // CustomThrust_Flag = (bool)log_msg->values[0];
    // CustomPWM_Flag = (bool)log_msg->values[1];
    // SafeModeEnable = (bool)log_msg->values[2];
    // AttCtrl_Flag = (bool)log_msg->values[3];

}
