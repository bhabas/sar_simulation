#include "SAR_DataConverter.h"

void SAR_DataConverter::RL_Data_Callback(const sar_msgs::RL_Data::ConstPtr &msg)
{

    k_ep = msg->k_ep;
    k_run = msg->k_run;
    n_rollouts = msg->n_rollouts;

    mu = msg->mu;
    sigma = msg->sigma;
    policy = msg->policy;

    reward = msg->reward;
    reward_vals = msg->reward_vals;

    vel_d = msg->vel_d;


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

    // CARTESIAN SPACE DATA
    StateData_msg.Pose = Pose;
    StateData_msg.Twist = Twist;
    StateData_msg.Accel = Accel;

    StateData_msg.Eul = Eul;



    // OPTICAL FLOW STATES
    StateData_msg.Tau = Tau;
    StateData_msg.Theta_x = Theta_x;
    StateData_msg.Theta_y = Theta_y;

    // PLANE RELATIVE STATES
    StateData_msg.D_perp = D_perp;
    StateData_msg.V_tx = V_rel.x;
    StateData_msg.V_ty = V_rel.y;
    StateData_msg.V_perp = V_rel.z;

    StateData_msg.V_mag_rel = V_mag_rel;
    StateData_msg.V_angle_rel = V_angle_rel;

    // OPTICAL FLOW STATE ESTIMATES
    StateData_msg.Tau_est = Tau_est;
    StateData_msg.Theta_x_est = Theta_x_est;
    StateData_msg.Theta_y_est = Theta_y_est;

    // STATE SETPOINTS
    StateData_msg.x_d = x_d;
    StateData_msg.v_d = v_d;
    StateData_msg.a_d = a_d;

    // CONTROL ACTIONS
    StateData_msg.FM = FM;
    StateData_msg.MotorThrusts = MotorThrusts;
    StateData_msg.MS_PWM = MS_PWM;

    // NEURAL NETWORK DATA
    StateData_msg.Policy_Trg_Action = Policy_Trg_Action;
    StateData_msg.Policy_Rot_Action = Policy_Rot_Action;


    // PUBLISH STATE DATA RECEIVED FROM CRAZYFLIE CONTROLLER
    StateData_Pub.publish(StateData_msg);
}

void SAR_DataConverter::Publish_TriggerData()
{

    ros::Duration Time_delta(Time_trg-Time_start);
    TriggerData_msg.header.stamp.sec = Time_delta.sec;
    TriggerData_msg.header.stamp.nsec = Time_delta.nsec;
    TriggerData_msg.Trg_flag = Trg_flag;


    // CARTESIAN SPACE DATA
    TriggerData_msg.Pose_trg = Pose_trg;
    TriggerData_msg.Twist_trg = Twist_trg;
    TriggerData_msg.Accel_trg = Accel_trg;
    TriggerData_msg.Eul_trg = Eul_trg;



    // OPTICAL FLOW
    TriggerData_msg.Tau_trg = Tau_trg;
    TriggerData_msg.Theta_x_trg = Theta_x_trg;
    TriggerData_msg.Theta_y_trg = Theta_y_trg;
    TriggerData_msg.D_perp_trg = D_perp_trg;

    // CONTROL ACTIONS
    TriggerData_msg.FM_trg = FM_trg;

    // NEURAL NETWORK DATA
    TriggerData_msg.Policy_Trg_Action_trg = Policy_Trg_Action_trg;
    TriggerData_msg.Policy_Rot_Action_trg = Policy_Rot_Action_trg;


    // PUBLISH STATE DATA RECEIVED FROM CRAZYFLIE CONTROLLER
    TriggerData_Pub.publish(TriggerData_msg);

}

void SAR_DataConverter::Publish_ImpactData()
{
    ros::Duration Time_delta(Time_impact-Time_start);
    ImpactData_msg.header.stamp.sec = Time_delta.sec;
    ImpactData_msg.header.stamp.nsec = Time_delta.nsec;

    ImpactData_msg.Impact_flag = Impact_flag;
    ImpactData_msg.BodyContact_flag = BodyContact_flag;
    ImpactData_msg.LegContact_flag = LegContact_flag;

    ImpactData_msg.Force_impact.x = impact_force_x;
    ImpactData_msg.Force_impact.y = impact_force_y;
    ImpactData_msg.Force_impact.z = impact_force_z;

    ImpactData_msg.Pose_impact = Pose_impact;
    ImpactData_msg.Twist_impact = Twist_impact;
    ImpactData_msg.Accel_impact = Accel_impact;
    ImpactData_msg.Eul_impact = Eul_impact;
    ImpactData_msg.Impact_Magnitude = impact_magnitude;

    ImpactData_msg.Pad_Connections = Pad_Connect_Sum;

    ImpactData_msg.Pad1_Contact = Pad1_Contact;
    ImpactData_msg.Pad2_Contact = Pad2_Contact;
    ImpactData_msg.Pad3_Contact = Pad3_Contact;
    ImpactData_msg.Pad4_Contact = Pad4_Contact;

    ImpactData_msg.Rot_Sum = Rot_Sum;

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

