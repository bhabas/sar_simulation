#include "CF_DataConverter.h"

void CF_DataConverter::Publish_StateData()
{
    // ===================
    //     FLIGHT DATA
    // ===================
    StateData_msg.header.stamp = Time;

    // CARTESIAN SPACE DATA
    StateData_msg.Pose = Pose;
    StateData_msg.Twist = Twist;

    StateData_msg.Eul = Eul;



    // OPTICAL FLOW
    StateData_msg.Tau = Tau;
    StateData_msg.OFx = OFx;
    StateData_msg.OFy = OFy;
    StateData_msg.RREV = RREV;
    StateData_msg.D_ceil = D_ceil;

    // STATE SETPOINTS
    StateData_msg.x_d = x_d;
    StateData_msg.v_d = v_d;
    StateData_msg.a_d = a_d;

    // CONTROL ACTIONS
    StateData_msg.FM = FM;
    StateData_msg.MS_PWM = MS_PWM;

    // NEURAL NETWORK DATA
    StateData_msg.NN_flip = NN_flip;
    StateData_msg.NN_policy = NN_policy;


    // PUBLISH STATE DATA RECEIVED FROM CRAZYFLIE CONTROLLER
    StateData_Pub.publish(StateData_msg);
}

void CF_DataConverter::Publish_FlipData()
{

    FlipData_msg.header.stamp = Time_tr;
    FlipData_msg.flip_flag = flip_flag;


    // CARTESIAN SPACE DATA
    FlipData_msg.Pose_tr = Pose_tr;
    FlipData_msg.Twist_tr = Twist_tr;
    FlipData_msg.Eul_tr = Eul_tr;



    // OPTICAL FLOW
    FlipData_msg.Tau_tr = Tau_tr;
    FlipData_msg.OFx_tr = OFx_tr;
    FlipData_msg.OFy_tr = OFy_tr;
    FlipData_msg.RREV_tr = RREV_tr;
    FlipData_msg.D_ceil_tr = D_ceil_tr;

    // CONTROL ACTIONS
    FlipData_msg.FM_tr = FM_tr;

    // NEURAL NETWORK DATA
    FlipData_msg.NN_tr_flip = NN_tr_flip;
    FlipData_msg.NN_tr_policy = NN_tr_policy;


    // PUBLISH STATE DATA RECEIVED FROM CRAZYFLIE CONTROLLER
    FlipData_Pub.publish(FlipData_msg);

}

void CF_DataConverter::Publish_MiscData()
{
    MiscData_msg.header.stamp = ros::Time::now();
    MiscData_msg.battery_voltage = 0.0;
    MiscData_Pub.publish(MiscData_msg);
}

void CF_DataConverter::Publish_ImpactData()
{
    ImpactData_msg.impact_flag = impact_flag;
    ImpactData_msg.BodyContact_flag = BodyContact_flag;
    ImpactData_msg.header.stamp = Time_impact;

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

void CF_DataConverter::CtrlData_Callback(const crazyflie_msgs::CtrlData &ctrl_msg)
{
    // ===================
    //     FLIGHT DATA
    // ===================
    Time = ros::Time::now();
    Pose = ctrl_msg.Pose;
    Twist = ctrl_msg.Twist;

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

    // OPTICAL FLOW
    Tau = ctrl_msg.Tau;
    OFx = ctrl_msg.OFx;
    OFy = ctrl_msg.OFy;
    RREV = ctrl_msg.RREV;
    D_ceil = ctrl_msg.D_ceil;

    // STATE SETPOINTS
    x_d = ctrl_msg.x_d;
    v_d = ctrl_msg.v_d;
    a_d = ctrl_msg.a_d;

    // CONTROL ACTIONS
    FM = ctrl_msg.FM;
    MS_PWM = ctrl_msg.MS_PWM;

    // NEURAL NETWORK DATA
    NN_flip = ctrl_msg.NN_flip;
    NN_policy = ctrl_msg.NN_policy;

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
    OFx_tr = ctrl_msg.OFx_tr;
    OFy_tr = ctrl_msg.OFy_tr;
    RREV_tr = ctrl_msg.RREV_tr;
    D_ceil_tr = ctrl_msg.D_ceil_tr;

    // CONTROLLER ACTIONS
    FM_tr = ctrl_msg.FM_flip;

    // NEURAL NETWORK DATA
    NN_tr_flip = ctrl_msg.NN_tr_flip;
    NN_tr_policy = ctrl_msg.NN_tr_policy;

}

void CF_DataConverter::RL_CMD_Callback(const crazyflie_msgs::RLCmd::ConstPtr &msg)
{
    if(msg->cmd_type == 0)
    {
        // RESET FLIP TIME
        OnceFlag_flip = false;
        Time_tr.sec = 0.0;
        Time_tr.nsec = 0.0;

        // RESET IMPACT TIME
        impact_flag = false;
        BodyContact_flag = false;
        OnceFlag_impact = false;
        Time_impact.sec = 0.0;
        Time_impact.nsec = 0.0;

        // RESET IMPACT VALUES
        Pose_impact.position.x = 0.0;
        Pose_impact.position.y = 0.0;
        Pose_impact.position.z = 0.0;

        Pose_impact.orientation.x = 0.0;
        Pose_impact.orientation.y = 0.0;
        Pose_impact.orientation.z = 0.0;
        Pose_impact.orientation.w = 0.0;

        Twist_impact.linear.x = 0.0;
        Twist_impact.linear.y = 0.0;
        Twist_impact.linear.z = 0.0;

        Twist_impact.angular.x = 0.0;
        Twist_impact.angular.y = 0.0;
        Twist_impact.angular.z = 0.0;

        Eul_impact.x = 0.0;
        Eul_impact.y = 0.0;
        Eul_impact.z = 0.0;

        // RESET MAX IMPACT FORCE
        impact_force_x = 0.0;
        impact_force_y = 0.0;
        impact_force_z = 0.0;

        // RESET PAD CONTACTS FLAGS
        Pad1_Contact = 0;
        Pad2_Contact = 0;
        Pad3_Contact = 0;
        Pad4_Contact = 0;

        Pad_Connections = 0;
                
    }


}

void CF_DataConverter::SurfaceFT_Sensor_Callback(const geometry_msgs::WrenchStamped::ConstPtr &msg)
{
    // RECORD MAX FORCE EXPERIENCED
    if (msg->wrench.force.x > impact_force_x){
        impact_force_x = msg->wrench.force.x;
    }
    if (msg->wrench.force.y > impact_force_y){
        impact_force_y = msg->wrench.force.y;
    }
    if (msg->wrench.force.z > impact_force_z){
        impact_force_z = msg->wrench.force.z;
    }

    // CHECK IF IMPACT FORCE THRESHOLD HAS BEEN CROSSED
    impact_force_resultant = sqrt(
        pow(msg->wrench.force.x,2) + 
        pow(msg->wrench.force.y,2) + 
        pow(msg->wrench.force.z,2));

    if (impact_force_resultant >= impact_thr && impact_flag == false){ 

        // LOCK IN STATE DATA WHEN IMPACT DETECTED
        impact_flag = true;

        // RECORD IMPACT STATE DATA FROM END OF CIRCULAR BUFFER WHEN IMPACT FLAGGED
        Time_impact = ros::Time::now();
        Pose_impact = Pose_impact_buff.front();
        Twist_impact = Twist_impact_buff.front();

        // PROCESS EULER ANGLES
        float quat_impact[4] = {
            (float)Pose_impact.orientation.x,
            (float)Pose_impact.orientation.y,
            (float)Pose_impact.orientation.z,
            (float)Pose_impact.orientation.w
        };
        float eul_impact[3];
        quat2euler(quat_impact,eul_impact);
        Eul_impact.x = eul_impact[0]*180/M_PI;
        Eul_impact.y = eul_impact[1]*180/M_PI;
        Eul_impact.z = eul_impact[2]*180/M_PI;

    }



}

// CONVERT QUATERNION TO EULER ANGLES (YZX NOTATION)
void CF_DataConverter::quat2euler(float quat[], float eul[]){

    float R11,R21,R31,R22,R23;
    float phi,theta,psi; // (phi=>x,theta=>y,psi=>z)

    // CALC NEEDED ROTATION MATRIX COMPONENTS FROM QUATERNION
    R11 = 1.0 - 2.0*(pow(quat[1],2) + pow(quat[2],2) );
    R21 = 2.0*(quat[0]*quat[1] + quat[2]*quat[3]);
    R31 = 2.0*(quat[0]*quat[2] - quat[1]*quat[3]);

    R22 = 1.0 - 2.0*( pow(quat[0],2) + pow(quat[2],2) );
    R23 = 2.0*(quat[1]*quat[2] - quat[0]*quat[3]);

    // CONVERT ROTATION MATRIX COMPONENTS TO EULER ANGLES (YZX NOTATION)
    phi = atan2(-R23, R22);
    theta = atan2(-R31, R11);
    psi = asin(R21);

    eul[0] = phi;   // X-axis
    eul[1] = theta; // Y-axis
    eul[2] = psi;   // Z-axis
}

void CF_DataConverter::MainLoop()
{
    int loopRate = 100;     // [Hz]
    int consoleRate = 20;   // [Hz]
    ros::Rate rate(loopRate);
    
    while(ros::ok)
    {   
        // DISPLAY CONSOLE AT CONSOLE_RATE FREQUENCY
        if (tick%(loopRate/consoleRate) == 0) {
            CF_DataConverter::consoleOuput();
        }

        // PUBLISH ORGANIZED DATA
        Publish_StateData();
        Publish_FlipData();
        Publish_ImpactData();
        Publish_MiscData();

        tick++;
        rate.sleep();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"CF_DataConverter_Node");
    ros::NodeHandle nh;
    CF_DataConverter CF_DC = CF_DataConverter(&nh);
    ros::spin();
    return 0;
}