#include "SAR_DataConverter.h"

void SAR_DataConverter::RL_Data_Callback(const crazyflie_msgs::RLData::ConstPtr &msg)
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

    StateData_msg.Eul = Eul;



    // OPTICAL FLOW STATES
    StateData_msg.Tau = Tau;
    StateData_msg.Theta_x = Theta_x;
    StateData_msg.Theta_y = Theta_y;
    StateData_msg.D_perp = D_perp;

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
    StateData_msg.Policy_Flip = Policy_Flip;
    StateData_msg.Policy_Action = Policy_Action;


    // PUBLISH STATE DATA RECEIVED FROM CRAZYFLIE CONTROLLER
    StateData_Pub.publish(StateData_msg);
}

void SAR_DataConverter::Publish_FlipData()
{

    ros::Duration Time_delta(Time_tr-Time_start);
    FlipData_msg.header.stamp.sec = Time_delta.sec;
    FlipData_msg.header.stamp.nsec = Time_delta.nsec;
    FlipData_msg.flip_flag = flip_flag;


    // CARTESIAN SPACE DATA
    FlipData_msg.Pose_tr = Pose_tr;
    FlipData_msg.Twist_tr = Twist_tr;
    FlipData_msg.Eul_tr = Eul_tr;



    // OPTICAL FLOW
    FlipData_msg.Tau_tr = Tau_tr;
    FlipData_msg.Theta_x_tr = Theta_x_tr;
    FlipData_msg.Theta_y_tr = Theta_y_tr;
    FlipData_msg.D_perp_tr = D_perp_tr;

    // CONTROL ACTIONS
    FlipData_msg.FM_tr = FM_tr;

    // NEURAL NETWORK DATA
    FlipData_msg.Policy_Flip_tr = Policy_Flip_tr;
    FlipData_msg.Policy_Action_tr = Policy_Action_tr;


    // PUBLISH STATE DATA RECEIVED FROM CRAZYFLIE CONTROLLER
    FlipData_Pub.publish(FlipData_msg);

}

void SAR_DataConverter::Publish_MiscData()
{
    MiscData_msg.header.stamp = ros::Time::now();
    MiscData_msg.battery_voltage = V_battery;
    MiscData_msg.Plane_Angle = Plane_Angle;
    MiscData_msg.Plane_Pos.x = Plane_Pos.x;
    MiscData_msg.Plane_Pos.y = Plane_Pos.y;
    MiscData_msg.Plane_Pos.z = Plane_Pos.z;


    
    MiscData_Pub.publish(MiscData_msg);
}

void SAR_DataConverter::Publish_ImpactData()
{
    ros::Duration Time_delta(Time_impact-Time_start);
    ImpactData_msg.header.stamp.sec = Time_delta.sec;
    ImpactData_msg.header.stamp.nsec = Time_delta.nsec;

    ImpactData_msg.impact_flag = impact_flag;
    ImpactData_msg.BodyContact_flag = BodyContact_flag;

    ImpactData_msg.Force_impact.x = impact_force_x;
    ImpactData_msg.Force_impact.y = impact_force_y;
    ImpactData_msg.Force_impact.z = impact_force_z;

    ImpactData_msg.Pose_impact = Pose_impact;
    ImpactData_msg.Twist_impact = Twist_impact;
    ImpactData_msg.Eul_impact = Eul_impact;

    ImpactData_msg.Pad_Connections = Pad_Connections;

    ImpactData_msg.Pad1_Contact = Pad1_Contact;
    ImpactData_msg.Pad2_Contact = Pad2_Contact;
    ImpactData_msg.Pad3_Contact = Pad3_Contact;
    ImpactData_msg.Pad4_Contact = Pad4_Contact;



    ImpactData_Pub.publish(ImpactData_msg);
}
