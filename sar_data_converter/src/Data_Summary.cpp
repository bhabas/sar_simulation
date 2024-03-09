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
    StateData_msg.Time.data.sec = Time_delta.sec;
    StateData_msg.Time.data.nsec = Time_delta.nsec;

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
    StateData_msg.D_perp_CR = D_perp_CR;
    StateData_msg.D_perp_CR_min = D_perp_CR_min;


    // OPTICAL FLOW STATES
    StateData_msg.Optical_Flow = Optical_Flow;
    StateData_msg.Optical_Flow_Cam = Optical_Flow_Cam;
    StateData_msg.Tau_CR = Tau_CR;

    // STATE SETPOINTS
    StateData_msg.x_d = x_d;
    StateData_msg.v_d = v_d;
    StateData_msg.a_d = a_d;

    // CONTROL ACTIONS
    StateData_msg.FM = FM;
    StateData_msg.MotorThrusts = MotorThrusts;
    StateData_msg.Motor_CMD = Motor_CMD;

    // POLICY ACTIONS
    StateData_msg.NN_Output = NN_Output;
    StateData_msg.a_Trg = a_Trg;
    StateData_msg.a_Rot = a_Rot;


    // PUBLISH STATE DATA RECEIVED FROM CONTROLLER
    StateData_Pub.publish(StateData_msg);
}

void SAR_DataConverter::Publish_TriggerData()
{

    ros::Duration Time_delta(Time_trg-Time_start);
    TriggerData_msg.Time_trg.data.sec = Time_delta.sec;
    TriggerData_msg.Time_trg.data.nsec = Time_delta.nsec;

    TriggerData_msg.Trg_Flag = Trg_Flag;


    // STATES WRT ORIGIN
    TriggerData_msg.Pose_B_O_trg = Pose_B_O_trg;
    TriggerData_msg.Twist_B_O_trg = Twist_B_O_trg;
    TriggerData_msg.Eul_B_O_trg = Eul_B_O_trg;

    TriggerData_msg.Vel_mag_B_O_trg = Vel_mag_B_O_trg;
    TriggerData_msg.Vel_angle_B_O_trg = Vel_angle_B_O_trg;

    // STATES WRT PLANE
    TriggerData_msg.Pose_P_B_trg = Pose_P_B_trg;
    TriggerData_msg.Twist_B_P_trg = Twist_B_P_trg;
    TriggerData_msg.Eul_P_B_trg = Eul_P_B_trg;

    TriggerData_msg.Vel_mag_B_P_trg = Vel_mag_B_P_trg;
    TriggerData_msg.Vel_angle_B_P_trg = Vel_angle_B_P_trg;
    TriggerData_msg.D_perp_trg = D_perp_trg;
    TriggerData_msg.D_perp_CR_trg = D_perp_CR_trg;

    // OPTICAL FLOW STATES
    TriggerData_msg.Optical_Flow_trg = Optical_Flow_trg;
    TriggerData_msg.Tau_CR_trg = Tau_CR_trg;

    // POLICY ACTIONS
    TriggerData_msg.NN_Output_trg = NN_Output_trg;
    TriggerData_msg.a_Trg_trg = a_Trg_trg;
    TriggerData_msg.a_Rot_trg = a_Rot_trg;

    TriggerData_Pub.publish(TriggerData_msg);
}

void SAR_DataConverter::Publish_ImpactData()
{

    ImpactData_msg.Impact_Flag = Impact_Flag;

    // ONBOARD IMPACT DATA
    ros::Duration Time_delta_OB(Time_impact_OB-Time_start);
    ImpactData_msg.Time_impact_OB.data.sec = Time_delta_OB.sec;
    ImpactData_msg.Time_impact_OB.data.nsec = Time_delta_OB.nsec;

    ImpactData_msg.Impact_Flag_OB = Impact_Flag_OB;

    ImpactData_msg.Pose_B_O_impact_OB = Pose_B_O_impact_OB;
    ImpactData_msg.Twist_B_P_impact_OB = Twist_B_P_impact_OB;
    ImpactData_msg.Eul_P_B_impact_OB = Eul_P_B_impact_OB;
    ImpactData_msg.Accel_B_O_Mag_impact_OB = Accel_B_O_Mag_impact_OB;

    // EXTERNAL IMPACT DATA
    ros::Duration Time_delta_Ext(Time_impact_Ext-Time_start);
    ImpactData_msg.Time_impact_Ext.data.sec = Time_delta_Ext.sec;
    ImpactData_msg.Time_impact_Ext.data.nsec = Time_delta_Ext.nsec;

    ImpactData_msg.Impact_Flag_Ext = Impact_Flag_Ext;
    ImpactData_msg.BodyContact_Flag = BodyContact_Flag;
    ImpactData_msg.ForelegContact_Flag = ForelegContact_Flag;
    ImpactData_msg.HindlegContact_Flag = HindlegContact_Flag;

    ImpactData_msg.Pose_B_O_impact_Ext = Pose_B_O_impact_Ext;
    ImpactData_msg.Eul_B_O_impact_Ext = Eul_B_O_impact_Ext;
    
    ImpactData_msg.Twist_B_P_impact_Ext = Twist_B_P_impact_Ext;
    ImpactData_msg.Eul_P_B_impact_Ext = Eul_P_B_impact_Ext;
    ImpactData_msg.Rot_Sum_impact_Ext = Rot_Sum_impact_Ext;

    // IMPACT FORCES
    ImpactData_msg.Force_impact.x = Force_Impact_x;
    ImpactData_msg.Force_impact.y = Force_Impact_y;
    ImpactData_msg.Force_impact.z = Force_Impact_z;
    ImpactData_msg.Impact_Magnitude = Impact_Magnitude;


    // STICKY PAD CONTACTS
    ImpactData_msg.Pad_Connections = Pad_Connections;

    ImpactData_msg.Pad1_Contact = Pad1_Contact;
    ImpactData_msg.Pad2_Contact = Pad2_Contact;
    ImpactData_msg.Pad3_Contact = Pad3_Contact;
    ImpactData_msg.Pad4_Contact = Pad4_Contact;


    ImpactData_Pub.publish(ImpactData_msg);
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

