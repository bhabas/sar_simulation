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


    // =================
    //     FLIP DATA
    // =================

    // CARTESIAN SPACE DATA
    if(ctrl_msg.flip_flag == true && OnceFlag_flip == false)
    {   
        Time_tr = ros::Time::now();
        OnceFlag_flip = true;

    }

    if(ctrl_msg.flip_flag == true && impact_flag == false)
    {
        double Time_delta = Time.toSec()-Time_prev.toSec();
        Rot_Sum += (Time_delta*Twist.angular.y)*180/M_PI;
        // printf("Val: %f\n",Rot_Sum);
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
    Camera_Sensor_Active = ctrl_msg.Camera_Sensor_Active;
}



void SAR_DataConverter::decompressXY(uint32_t xy, float xy_arr[])
{
    uint16_t xd = ((uint32_t)xy >> 16) & 0xFFFF;    // Shift out y-value bits
    xy_arr[0] = ((float)xd - 32767.0f)*1e-3;        // Convert to normal value

    uint16_t yd = (uint32_t)xy & 0xFFFF;            // Save only y-value bits
    xy_arr[1] = ((float)yd - 32767.0f)*1e-3;

}



void SAR_DataConverter::log1_Callback(const sar_msgs::GenericLogData::ConstPtr &log1_msg)
{
    // ===================
    //     FLIGHT DATA
    // ===================
    Time = ros::Time::now();

    // POSITION
    float xy_arr[2];
    decompressXY(log1_msg->values[0],xy_arr);

    Pose.position.x = xy_arr[0];
    Pose.position.y = xy_arr[1];
    Pose.position.z = log1_msg->values[1]*1e-3;

    // VELOCITY
    float vxy_arr[2];
    decompressXY(log1_msg->values[2],vxy_arr);
    
    Twist.linear.x = vxy_arr[0];
    Twist.linear.y = vxy_arr[1];
    Twist.linear.z = log1_msg->values[3]*1e-3;
    Vel_mag = sqrt(pow(Twist.linear.x,2)+pow(Twist.linear.y,2)+pow(Twist.linear.z,2));
    Phi = atan2(Twist.linear.z,Twist.linear.x)*180/M_PI;
    Alpha = atan2(Twist.linear.y,Twist.linear.x)*180/M_PI;

    // ORIENTATION
    float quat[4];
    uint32_t quatZ = (uint32_t)log1_msg->values[4];
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
    decompressXY(log1_msg->values[5],wxy_arr);
    
    Twist.angular.x = wxy_arr[0]*10;
    Twist.angular.y = wxy_arr[1]*10;


    // OPTICAL FLOW
    float OF_xy_arr[2];
    decompressXY(log1_msg->values[6],OF_xy_arr);
    
    Theta_x = OF_xy_arr[0];
    Theta_y = OF_xy_arr[1];
    Tau = log1_msg->values[7]*1e-3;

}

void SAR_DataConverter::log2_Callback(const sar_msgs::GenericLogData::ConstPtr &log2_msg)
{
    // ANGULAR VELOCITY (Z)
    Twist.angular.z = log2_msg->values[0]*1e-3;

    // CEILING DISTANCE
    D_perp = log2_msg->values[1]*1e-3;

    // DECOMPRESS THRUST/MOMENT MOTOR VALUES [g]
    float FM_z[2];
    float M_xy[2];

    decompressXY(log2_msg->values[2],FM_z);
    decompressXY(log2_msg->values[3],M_xy); 

    FM = {FM_z[0],M_xy[0],M_xy[1],FM_z[1]}; // [F,Mx,My,Mz]


    // MOTOR PWM VALUES
    float MS_PWM12[2];
    float MS_PWM34[2];

    decompressXY(log2_msg->values[4],MS_PWM12);
    decompressXY(log2_msg->values[5],MS_PWM34);

    MS_PWM = {
        (uint16_t)round(MS_PWM12[0]*2.0e3),
        (uint16_t)round(MS_PWM12[1]*2.0e3), 
        (uint16_t)round(MS_PWM34[0]*2.0e3),
        (uint16_t)round(MS_PWM34[1]*2.0e3)
    };
    
    // NEURAL NETWORK VALUES
    float NN_FP[2];
    decompressXY(log2_msg->values[6],NN_FP);
    Policy_Trg_Action = NN_FP[0];
    Policy_Flip_Action = NN_FP[1];

    // OTHER MISC INFO
    flip_flag = log2_msg->values[7];
    if(flip_flag == true && OnceFlag_flip == false)
    {
        Time_tr = ros::Time::now();
        OnceFlag_flip = true;
    }

    // V_battery = 3.5 + (log2_msg->values[8]/256)*(4.2-3.5);


}

void SAR_DataConverter::log3_Callback(const sar_msgs::GenericLogData::ConstPtr &log3_msg)
{
    // POSITION SETPOINTS
    float xd_xy[2];
    decompressXY(log3_msg->values[0],xd_xy);

    x_d.x = xd_xy[0];
    x_d.y = xd_xy[1];
    x_d.z = log3_msg->values[1]*1e-3;
   
    // VELOCITY SETPOINTS
    float vd_xy[2];
    decompressXY(log3_msg->values[2],vd_xy);

    v_d.x = vd_xy[0];
    v_d.y = vd_xy[1];
    v_d.z = log3_msg->values[3]*1e-3;

    // ACCELERATION SETPOINTS
    float ad_xy[2];
    decompressXY(log3_msg->values[4],ad_xy);

    a_d.x = ad_xy[0];
    a_d.y = ad_xy[1];
    a_d.z = log3_msg->values[5]*1e-3;

    // MOTOR THRUST VALUES
    float M_thrust12[2];
    float M_thrust34[2];

    decompressXY(log3_msg->values[6],M_thrust12);
    decompressXY(log3_msg->values[7],M_thrust34);

    MotorThrusts = {M_thrust12[0],M_thrust12[1],M_thrust34[0],M_thrust34[1]};

    
}

void SAR_DataConverter::log4_Callback(const sar_msgs::GenericLogData::ConstPtr &log4_msg)
{
    // FLIP TRIGGER - POSITION
    Pose_tr.position.x = NAN;
    Pose_tr.position.y = NAN;
    Pose_tr.position.z = log4_msg->values[0]*1e-3;

    // FLIP TRIGGER - CEILING DISTANCE
    D_perp_tr = log4_msg->values[1]*1e-3;

    // FLIP TRIGGER - VELOCITY
    float vxy_arr[2];
    decompressXY(log4_msg->values[2],vxy_arr);
    
    Twist_tr.linear.x = vxy_arr[0];
    Twist_tr.linear.y = vxy_arr[1];
    Twist_tr.linear.z = log4_msg->values[3]*1e-3;
    

    // FLIP TRIGGER - ORIENTATION
    float quat_tr[4];
    uint32_t quatZ = (uint32_t)log4_msg->values[4];
    quatdecompress(quatZ,quat_tr);

    Pose_tr.orientation.x = quat_tr[0];
    Pose_tr.orientation.y = quat_tr[1];
    Pose_tr.orientation.z = quat_tr[2];
    Pose_tr.orientation.w = quat_tr[3]; 

    // FLIP TRIGGER - ANGULAR VELOCITY
    float wxy_arr[2];
    decompressXY(log4_msg->values[5],wxy_arr);
    
    Twist_tr.angular.x = wxy_arr[0];
    Twist_tr.angular.y = wxy_arr[1];
    Twist_tr.angular.z = NAN;

    // FLIP TRIGGER - OPTICAL FLOW
    float OF_xy_arr[2];
    decompressXY(log4_msg->values[6],OF_xy_arr);
    
    Theta_x_tr = OF_xy_arr[0];
    Theta_y_tr = OF_xy_arr[1];
    Tau_tr = log4_msg->values[7]*1e-3;

}

void SAR_DataConverter::log5_Callback(const sar_msgs::GenericLogData::ConstPtr &log5_msg)
{
    Pos_Ctrl_Flag = log5_msg->values[0];
    Vel_Ctrl_Flag = log5_msg->values[1];
    Motorstop_Flag = log5_msg->values[2];
    Moment_Flag = log5_msg->values[3];
    Tumbled_Flag = log5_msg->values[4];
    Tumble_Detection = log5_msg->values[5];
    Policy_Armed_Flag =log5_msg->values[6];

}

void SAR_DataConverter::log6_Callback(const sar_msgs::GenericLogData::ConstPtr &log6_msg)
{
    V_battery = log6_msg->values[0];


}
