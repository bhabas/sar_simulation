#include "SAR_DataConverter.h"

void SAR_DataConverter::RL_Data_Callback(const sar_msgs::RL_Data::ConstPtr &msg)
{

    K_ep = msg->K_ep;
    K_run = msg->K_run;
    n_rollouts = msg->n_rollouts;

    mu = msg->mu;
    sigma = msg->sigma;
    // policy = msg->policy;

    reward = msg->reward;
    reward_vals = msg->reward_vals;

    // vel_d = msg->vel_d;


    if(msg->trialComplete_flag == true)
    {
        Time_start = ros::Time::now();
    }
}


void SAR_DataConverter::Publish_StateData()
{
    // ===================
    //     FLIGHT DATA
    // ===================
    ros::Duration Time_delta(Time-Time_start);
    StateData_msg.header.stamp.sec = Time_delta.sec;
    StateData_msg.header.stamp.nsec = Time_delta.nsec;

    // STATES WRT ORIGIN
    StateData_msg.Pose_B_O = Pose_B_O;
    StateData_msg.Twist_B_O = Twist_B_O;
    StateData_msg.Accel_B_O = Accel_B_O;
    StateData_msg.Eul_B_O = Eul_B_O;
    StateData_msg.Accel_B_O_Mag = Accel_B_O_Mag;

    // STATES WRT PLANE
    StateData_msg.Pose_P_B = Pose_P_B;
    StateData_msg.Twist_B_P = Twist_B_P;
    StateData_msg.Eul_P_B = Eul_P_B;
    StateData_msg.Vel_mag_B_P = Vel_mag_B_P;
    StateData_msg.Vel_angle_B_P = Vel_angle_B_P;
    StateData_msg.D_perp = D_perp;


    // OPTICAL FLOW STATES
    StateData_msg.Optical_Flow = Optical_Flow;
    StateData_msg.Optical_Flow_Cam = Optical_Flow_Cam;

    // STATE SETPOINTS
    StateData_msg.x_d = x_d;
    StateData_msg.v_d = v_d;
    StateData_msg.a_d = a_d;

    // CONTROL ACTIONS
    StateData_msg.FM = FM;
    StateData_msg.MotorThrusts = MotorThrusts;
    StateData_msg.MS_PWM = MS_PWM;

    // POLICY ACTIONS
    StateData_msg.Policy_Trg_Action = Policy_Trg_Action;
    StateData_msg.Policy_Rot_Action = Policy_Rot_Action;


    // PUBLISH STATE DATA RECEIVED FROM CONTROLLER
    StateData_Pub.publish(StateData_msg);
}

void SAR_DataConverter::Publish_TriggerData()
{

    ros::Duration Time_delta(Time_trg-Time_start);
    TriggerData_msg.header.stamp.sec = Time_delta.sec;
    TriggerData_msg.header.stamp.nsec = Time_delta.nsec;

    TriggerData_msg.Trg_Flag = Trg_Flag;


    // STATES WRT ORIGIN
    TriggerData_msg.Pose_B_O_trg = Pose_B_O_trg;
    TriggerData_msg.Twist_B_O_trg = Twist_B_O_trg;
    TriggerData_msg.Eul_B_O_trg = Eul_B_O_trg;

    // STATES WRT PLANE
    TriggerData_msg.Pose_P_B_trg = Pose_P_B_trg;
    TriggerData_msg.Twist_B_P_trg = Twist_B_P_trg;
    TriggerData_msg.Eul_P_B_trg = Eul_P_B_trg;

    // OPTICAL FLOW STATES
    TriggerData_msg.Optical_Flow_trg = Optical_Flow_trg;


    // POLICY ACTIONS
    TriggerData_msg.Policy_Trg_Action_trg = Policy_Trg_Action_trg;
    TriggerData_msg.Policy_Rot_Action_trg = Policy_Rot_Action_trg;

    TriggerData_Pub.publish(TriggerData_msg);
}

void SAR_DataConverter::Publish_ImpactData()
{
    // ros::Duration Time_delta(Time_impact-Time_start);
    // ImpactData_msg.header.stamp.sec = Time_delta.sec;
    // ImpactData_msg.header.stamp.nsec = Time_delta.nsec;

    // ImpactData_msg.Impact_Flag_Ext = Impact_Flag_Ext;
    // ImpactData_msg.BodyContact_flag = BodyContact_flag;
    // ImpactData_msg.LegContact_flag = LegContact_flag;

    // ImpactData_msg.Force_impact.x = impact_force_x;
    // ImpactData_msg.Force_impact.y = impact_force_y;
    // ImpactData_msg.Force_impact.z = impact_force_z;

    // ImpactData_msg.Pose_impact = Pose_impact;
    // ImpactData_msg.Twist_impact = Twist_impact;
    // ImpactData_msg.Accel_impact = Accel_impact;
    // ImpactData_msg.Eul_impact = Eul_impact;
    // ImpactData_msg.Impact_Magnitude = impact_magnitude;

    // ImpactData_msg.Pad_Connections = Pad_Connect_Sum;

    // ImpactData_msg.Pad1_Contact = Pad1_Contact;
    // ImpactData_msg.Pad2_Contact = Pad2_Contact;
    // ImpactData_msg.Pad3_Contact = Pad3_Contact;
    // ImpactData_msg.Pad4_Contact = Pad4_Contact;

    // ImpactData_msg.Rot_Sum = Rot_Sum;

    // ImpactData_Pub.publish(ImpactData_msg);
}

void SAR_DataConverter::Publish_MiscData()
{
    MiscData_msg.header.stamp = ros::Time::now();
    MiscData_msg.battery_voltage = V_battery;
    MiscData_msg.Plane_Angle = Plane_Angle_deg;
    MiscData_msg.Plane_Pos.x = Plane_Pos.x;
    MiscData_msg.Plane_Pos.y = Plane_Pos.y;
    MiscData_msg.Plane_Pos.z = Plane_Pos.z;


    
    MiscData_Pub.publish(MiscData_msg);
}

