#include "CF_DataConverter.h"

void CF_DataConverter::Publish_StateData()
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
    StateData_msg.MotorThrusts = MotorThrusts;
    StateData_msg.MS_PWM = MS_PWM;

    // NEURAL NETWORK DATA
    StateData_msg.NN_flip = NN_flip;
    StateData_msg.NN_policy = NN_policy;


    // PUBLISH STATE DATA RECEIVED FROM CRAZYFLIE CONTROLLER
    StateData_Pub.publish(StateData_msg);
}

void CF_DataConverter::Publish_FlipData()
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
    MiscData_msg.battery_voltage = V_battery;
    MiscData_Pub.publish(MiscData_msg);
}

void CF_DataConverter::Publish_ImpactData()
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
    MotorThrusts = ctrl_msg.MotorThrusts;
    MS_PWM = ctrl_msg.MS_PWM;

    // RL POLICY DATA
    Tau_thr = ctrl_msg.Tau_thr;
    G1 = ctrl_msg.G1;

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

void CF_DataConverter::CtrlDebug_Callback(const crazyflie_msgs::CtrlDebug &ctrl_msg)
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

        // RESET SIM SPEED
        CF_DataConverter::adjustSimSpeed(SIM_SPEED);
        SLOWDOWN_TYPE = 0;

        
                
    }

    if(msg->cmd_type == 6)
    {
        CF_DataConverter::LoadParams();
    }

    if(msg->cmd_type == 11)
    {
        if(msg->cmd_flag == 0)
        {
            Sticky_Flag = false;
        }

        if(msg->cmd_flag == 1)
        {
            Sticky_Flag = true;
        }

        CF_DataConverter::activateStickyFeet();

    }

    if(msg->cmd_type == 101)
    {

    }


}

void CF_DataConverter::RL_Data_Callback(const crazyflie_msgs::RLData::ConstPtr &msg)
{

    k_ep = msg->k_ep;
    k_run = msg->k_run;
    n_rollouts = msg->n_rollouts;

    mu = msg->mu;
    sigma = msg->sigma;
    policy = msg->policy;

    reward = msg->reward;
    reward_inputs = msg->reward_inputs;

    vel_d = msg->vel_d;

    runComplete_flag = msg->runComplete_flag;

    if(msg->trialComplete_flag == true)
    {
        Time_start = ros::Time::now();
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

// MARK IF CF BODY COLLIDES WITH CEILING
void CF_DataConverter::Surface_Contact_Callback(const gazebo_msgs::ContactsState &msg)
{
    // CYCLE THROUGH VECTOR OF CONTACT MESSAGES
    for (int i=0; i<msg.states.size(); i++)
    {
        // IF CONTACT MSG MATCHES BODY COLLISION STR THEN TURN ON BODY_CONTACT_FLAG 
        if(BodyContact_flag == false && strcmp(msg.states[i].collision1_name.c_str(),BodyCollision_str.c_str()) == 0)
        {
            BodyContact_flag = true;
        }  
    }
}

// MARK IF PAD CONTACT HAS OCCURED AND SUM NUMBER OF PAD CONTACTS
void CF_DataConverter::Pad_Connections_Callback(const crazyflie_msgs::PadConnect &msg)
{
    
    if(msg.Pad1_Contact == 1) Pad1_Contact = 1;
    if(msg.Pad2_Contact == 1) Pad2_Contact = 1;
    if(msg.Pad3_Contact == 1) Pad3_Contact = 1;
    if(msg.Pad4_Contact == 1) Pad4_Contact = 1;
    Pad_Connections = Pad1_Contact + Pad2_Contact + Pad3_Contact + Pad4_Contact;

}

// CHECK IF SIM SPEED NEEDS TO BE ADJUSTED
void CF_DataConverter::checkSlowdown()
{   
    // SIMULATION SLOWDOWN
    if(LANDING_SLOWDOWN_FLAG==true && tick >= 500){

        // WHEN CLOSE TO THE CEILING REDUCE SIM SPEED
        if(D_ceil<=0.5 && SLOWDOWN_TYPE == 0){
            
            CF_DataConverter::adjustSimSpeed(SIM_SLOWDOWN_SPEED);
            SLOWDOWN_TYPE = 1;
        }

        // IF IMPACTED CEILING OR FALLING AWAY, INCREASE SIM SPEED TO DEFAULT
        if(impact_flag == true && SLOWDOWN_TYPE == 1)
        {
            CF_DataConverter::adjustSimSpeed(SIM_SPEED);
            SLOWDOWN_TYPE = 2; // (Don't call adjustSimSpeed more than once)
        }
        else if(Twist.linear.z <= -0.5 && SLOWDOWN_TYPE == 1){
            CF_DataConverter::adjustSimSpeed(SIM_SPEED);
            SLOWDOWN_TYPE = 2;
        }
    
    }

}

// CHANGES REAL TIME FACTOR FOR THE SIMULATION (LOWER = SLOWER)
void CF_DataConverter::adjustSimSpeed(float speed_mult)
{
    gazebo_msgs::SetPhysicsProperties srv;
    srv.request.time_step = 0.001;
    srv.request.max_update_rate = (int)(speed_mult/0.001);


    geometry_msgs::Vector3 gravity_vec;
    gravity_vec.x = 0.0;
    gravity_vec.y = 0.0;
    gravity_vec.z = -9.8066;
    srv.request.gravity = gravity_vec;

    gazebo_msgs::ODEPhysics ode_config;
    ode_config.auto_disable_bodies = false;
    ode_config.sor_pgs_precon_iters = 0;
    ode_config.sor_pgs_iters = 50;
    ode_config.sor_pgs_w = 1.3;
    ode_config.sor_pgs_rms_error_tol = 0.0;
    ode_config.contact_surface_layer = 0.001;
    ode_config.contact_max_correcting_vel = 0.0;
    ode_config.cfm = 0.0;
    ode_config.erp = 0.2;
    ode_config.max_contacts = 20;

    srv.request.ode_config = ode_config;

    GZ_SimSpeed_Client.call(srv);
}

void CF_DataConverter::activateStickyFeet()
{
    if(MODEL_NAME != "crazyflie_BaseModel")
    {
        crazyflie_msgs::activateSticky srv;
        srv.request.stickyFlag = Sticky_Flag;

        ros::service::call("/activate_Sticky_Pad_1", srv);
        ros::service::call("/activate_Sticky_Pad_2", srv);
        ros::service::call("/activate_Sticky_Pad_3", srv);
        ros::service::call("/activate_Sticky_Pad_4", srv);
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

void CF_DataConverter::consoleOuput()
{
    system("clear");
    printf("t: %.4f V: %.3f\n",(Time-Time_start).toSec(),V_battery);
    printf("DataType: %s \t Model: %s\n",DATA_TYPE.c_str(),MODEL_NAME.c_str());
    printf("\n");

    printf("==== Flags ====\n");
    printf("Motorstop:\t%u  Flip_flag:\t  %u  Pos Ctrl:\t    %u \n",Motorstop_Flag, flip_flag, Pos_Ctrl_Flag);
    printf("Traj Active:\t%u  Impact_flag:\t  %u  Vel Ctrl:\t    %u \n",Traj_Active_Flag,impact_flag,Vel_Ctrl_Flag);
    printf("Policy_type:\t%u  Tumble Detect: %u  Moment_Flag:   %u \n",POLICY_TYPE,Tumble_Detection,Moment_Flag);
    printf("Policy_armed:\t%u  Tumbled:\t  %u  Slowdown_type: %u\n",Policy_Armed_Flag,Tumbled_Flag,SLOWDOWN_TYPE);
    printf("Sticky_flag:\t%u\n",Sticky_Flag);
    printf("\n");


    printf("==== System States ====\n");
    printf("Pos [m]:\t %.3f  %.3f  %.3f\n",Pose.position.x,Pose.position.y,Pose.position.z);
    printf("Vel [m/s]:\t %.3f  %.3f  %.3f\n",Twist.linear.x,Twist.linear.y,Twist.linear.z);
    printf("Omega [rad/s]:\t %.3f  %.3f  %.3f\n",Twist.angular.x,Twist.angular.y,Twist.angular.z);
    printf("Eul [deg]:\t %.3f  %.3f  %.3f\n",Eul.x,Eul.y,Eul.z);
    printf("\n");

    printf("Tau: %.3f \tOFx: %.3f \tOFy: %.3f \tRREV: %.3f\n",Tau,OFx,OFy,RREV);
    printf("D_ceil: %.3f\n",D_ceil);
    printf("\n");


    printf("==== Setpoints ====\n");
    printf("x_d: %.3f  %.3f  %.3f\n",x_d.x,x_d.y,x_d.z);
    printf("v_d: %.3f  %.3f  %.3f\n",v_d.x,v_d.y,v_d.z);
    printf("a_d: %.3f  %.3f  %.3f\n",a_d.x,a_d.y,a_d.z);
    printf("\n");

    
    printf("==== Policy Values ====\n");
    printf("RL: \n");
    printf("Tau_thr: %.3f \tG1: %.3f \tG2: %.3f\n",Tau_thr,G1,0.0);
    printf("\n");

    printf("NN_Outputs: \n");
    printf("NN_Flip: %.3f \tNN_Policy: %.3f \n",NN_flip,NN_policy);
    printf("\n");

    printf("==== Flip Trigger Values ====\n");
    printf("Tau_tr:     %.3f \tNN_tr_Flip:    %.3f \n",Tau_tr,NN_tr_flip);
    printf("OFy_tr:     %.3f \tNN_tr_Policy:  %.3f \n",OFy_tr,NN_tr_policy);
    printf("D_ceil_tr:  %.3f \n",D_ceil_tr);
    printf("\n");

    printf("==== Controller Actions ====\n");
    printf("FM [N/N*mm]: %.3f  %.3f  %.3f  %.3f\n",FM[0],FM[1],FM[2],FM[3]);
    printf("Motor Thrusts [g]: %.3f  %.3f  %.3f  %.3f\n",MotorThrusts[0],MotorThrusts[1],MotorThrusts[2],MotorThrusts[3]);
    printf("MS_PWM: %u  %u  %u  %u\n",MS_PWM[0],MS_PWM[1],MS_PWM[2],MS_PWM[3]);
    printf("\n");


    printf("=== Parameters ====\n");
    printf("Kp_P: %.3f  %.3f  %.3f \t",P_kp_xy,P_kp_xy,P_kp_z);
    printf("Kp_R: %.3f  %.3f  %.3f \n",R_kp_xy,R_kp_xy,R_kp_z);

    printf("Kd_P: %.3f  %.3f  %.3f \t",P_kd_xy,P_kd_xy,P_kd_z);
    printf("Kd_R: %.3f  %.3f  %.3f \n",R_kd_xy,R_kd_xy,R_kd_z);

    printf("Ki_P: %.3f  %.3f  %.3f \t",P_ki_xy,P_ki_xy,P_ki_z);
    printf("Ki_R: %.3f  %.3f  %.3f \n",R_ki_xy,R_ki_xy,R_ki_z);
    printf("======\n");
}


void CF_DataConverter::MainLoop()
{
    int loopRate = 100;     // [Hz]
    ros::Rate rate(loopRate);


    
    while(ros::ok)
    {   


        // DISPLAY CONSOLE AT CONSOLE_RATE FREQUENCY
        if (RATE_DO_EXECUTE(CONSOLE_RATE, tick))
        {
            // CF_DataConverter::consoleOuput();

        }

        if (RATE_DO_EXECUTE(LOGGING_RATE, tick))
        {
            if(Logging_Flag == true)
            {
                append_CSV_states();
            }
        }

        CF_DataConverter::checkSlowdown();

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