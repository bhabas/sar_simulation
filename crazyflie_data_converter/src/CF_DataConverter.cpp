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
    Camera_Sensor_Active = ctrl_msg.Camera_Sensor_Active;
}

bool CF_DataConverter::CMD_CF_DC_Callback(crazyflie_msgs::RLCmd::Request &req, crazyflie_msgs::RLCmd::Response &res)
{
    // PASS COMMAND VALUES TO CONTROLLER AND PASS LOCAL ACTIONS
    CF_DataConverter::Send_Cmd2Ctrl(req);
    res.srv_Success = true;
    return res.srv_Success;
}

bool CF_DataConverter::CMD_Dashboard_Callback(crazyflie_msgs::RLCmd::Request &req, crazyflie_msgs::RLCmd::Response &res)
{
    // PASS COMMAND VALUES TO CONTROLLER AND PASS LOCAL ACTIONS
    CF_DataConverter::Send_Cmd2Ctrl(req);
    res.srv_Success = true;
    return res.srv_Success;
}

bool CF_DataConverter::Send_Cmd2Ctrl(crazyflie_msgs::RLCmd::Request &req)
{
    if(req.cmd_type == 0)
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

        if (DATA_TYPE.compare("SIM") == 0)
        {
            // RESET SIM SPEED
            CF_DataConverter::adjustSimSpeed(SIM_SPEED);
            SLOWDOWN_TYPE = 0;
        }

    }

    if(req.cmd_type == 8)
    {
        // RL POLICY DATA
        Tau_thr = req.cmd_vals.x;
        G1 = req.cmd_vals.y;

    }

    if(req.cmd_type == 21) // UPDATE PARAMS IN CF_DC 
    {
        CF_DataConverter::LoadParams();
    }

    if(req.cmd_type == 92) // STICKY FEET
    {
        if (DATA_TYPE.compare("SIM") == 0)
        {
            if(req.cmd_flag == 0)
            {
                Sticky_Flag = false;
            }
            else
            {
                Sticky_Flag = true;
            }
            
            CF_DataConverter::activateStickyFeet();
        }
    }

    // SIMULATION:
    // SEND COMMAND VALUES TO CONTROLLER
    crazyflie_msgs::RLCmd srv;
    srv.request = req;
    CMD_Client.call(srv);


    // EXPERIMENT: 
    // BROADCAST CMD VALUES AS ROS MESSAGE
    // (SO CRAZYSWARM CAN PASS MSGS FROM BOTH DASHBOARD AND ENV FILE)
    crazyflie_msgs::GTC_Cmd cmd_msg;
    cmd_msg.cmd_type = req.cmd_type;
    cmd_msg.cmd_vals = req.cmd_vals;
    cmd_msg.cmd_flag = req.cmd_flag;
    CMD_Pub.publish(cmd_msg);

    return srv.response.srv_Success; // Return if service request successful (true/false)
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

    vel_d = msg->vel_d;


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
    if(LANDING_SLOWDOWN_FLAG==true){

        // WHEN CLOSE TO THE CEILING REDUCE SIM SPEED
        if(D_perp<=0.5 && SLOWDOWN_TYPE == 0){
            
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
    if(MODEL_NAME != "crazyflie_Base_Model")
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
    printf("Motorstop:\t%u  Flip_flag:\t  %u  Pos Ctrl:\t    %u  Cam_Est:\t  %u\n",Motorstop_Flag, flip_flag, Pos_Ctrl_Flag,Camera_Sensor_Active);
    printf("Traj Active:\t%u  Impact_flag:\t  %u  Vel Ctrl:\t    %u \n",Traj_Active_Flag,impact_flag,Vel_Ctrl_Flag);
    printf("Policy_armed:\t%u  Tumble Detect: %u  Moment_Flag:   %u \n",Policy_Armed_Flag,Tumble_Detection,Moment_Flag);
    printf("Sticky_flag:\t%u  Tumbled:\t  %u  Slowdown_type: %u\n",Sticky_Flag,Tumbled_Flag,SLOWDOWN_TYPE);
    printf("\n");

    printf("==== System States ====\n");
    printf("Pos [m]:\t %.3f  %.3f  %.3f\n",Pose.position.x,Pose.position.y,Pose.position.z);
    printf("Vel [m/s]:\t %.3f  %.3f  %.3f\n",Twist.linear.x,Twist.linear.y,Twist.linear.z);
    printf("Omega [rad/s]:\t %.3f  %.3f  %.3f\n",Twist.angular.x,Twist.angular.y,Twist.angular.z);
    printf("Eul [deg]:\t %.3f  %.3f  %.3f\n",Eul.x,Eul.y,Eul.z);
    printf("\n");
    printf("Vel [mag,phi,alph]: %.2f %.2f %.2f\n",Vel_mag,Phi,Alpha);
    printf("\n");

    printf("Tau: %.3f \t%sx: %.3f \t%sy: %.3f\n",Tau,theta,Theta_x,theta,Theta_y);
    printf("Tau_est: %.3f \t%sx_est: %.3f \t%sy_est: %.3f\n",Tau_est,theta,Theta_x_est,theta,Theta_y_est);
    printf("D_perp: %.3f\n",D_perp);
    printf("\n");

    printf("==== Policy: %s ====\n",POLICY_TYPE.c_str());
    if (strcmp(POLICY_TYPE.c_str(),"PARAM_OPTIM") == 0)
    {
        printf("Tau_thr: %.3f \tMy: %.3f\n",Tau_thr,G1);
        printf("\n");
    }
    else if (strcmp(POLICY_TYPE.c_str(),"SVL_POLICY") == 0)
    {
        printf("Policy_Flip: %.3f \tPolicy_Action: %.3f \n",Policy_Flip,Policy_Action);
        printf("\n");
    }
    else if (strcmp(POLICY_TYPE.c_str(),"DEEP_RL") == 0)
    {
        printf("Stuff: %.3f \tStuff: %.3f \n",Policy_Flip,Policy_Action);
        printf("\n");
    }


    printf("==== Flip Trigger Values ====\n");
    printf("Tau_tr:     %.3f \tPolicy_Flip_tr:    %.3f \n",Tau_tr,Policy_Flip_tr);
    printf("%sx_tr:     %.3f \tPolicy_Action_tr:  %.3f \n",theta,Theta_x_tr,Policy_Action_tr);
    printf("D_perp_tr:  %.3f \n",D_perp_tr);
    printf("\n");

    printf("==== Setpoints ====\n");
    printf("x_d: %.3f  %.3f  %.3f\n",x_d.x,x_d.y,x_d.z);
    printf("v_d: %.3f  %.3f  %.3f\n",v_d.x,v_d.y,v_d.z);
    printf("a_d: %.3f  %.3f  %.3f\n",a_d.x,a_d.y,a_d.z);
    printf("\n");

    printf("==== Controller Actions ====\n");
    printf("FM [N/N*mm]: %.3f  %.3f  %.3f  %.3f\n",FM[0],FM[1],FM[2],FM[3]);
    printf("Motor Thrusts [g]: %.3f  %.3f  %.3f  %.3f\n",MotorThrusts[0],MotorThrusts[1],MotorThrusts[2],MotorThrusts[3]);
    printf("MS_PWM: %u  %u  %u  %u\n",MS_PWM[0],MS_PWM[1],MS_PWM[2],MS_PWM[3]);
    printf("\n");


    printf("=== Controller Gains ====\n");
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
    int loopRate = 1000;     // [Hz]
    ros::Rate rate(loopRate);


    
    while(ros::ok)
    {   


        // DISPLAY CONSOLE AT CONSOLE_RATE FREQUENCY
        if (RATE_DO_EXECUTE(CONSOLE_RATE, tick))
        {
            CF_DataConverter::consoleOuput();

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