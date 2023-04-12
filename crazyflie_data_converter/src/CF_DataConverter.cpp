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

bool CF_DataConverter::CMD_CF_DC_Callback(crazyflie_msgs::GTC_Cmd_srv::Request &req, crazyflie_msgs::GTC_Cmd_srv::Response &res)
{
    // PASS COMMAND VALUES TO CONTROLLER AND PASS LOCAL ACTIONS
    CF_DataConverter::Send_Cmd2Ctrl(req);
    res.srv_Success = true;
    return res.srv_Success;
}

bool CF_DataConverter::CMD_Dashboard_Callback(crazyflie_msgs::GTC_Cmd_srv::Request &req, crazyflie_msgs::GTC_Cmd_srv::Response &res)
{
    // PASS COMMAND VALUES TO CONTROLLER AND PASS LOCAL ACTIONS
    CF_DataConverter::Send_Cmd2Ctrl(req);
    res.srv_Success = true;
    return res.srv_Success;
}

bool CF_DataConverter::Send_Cmd2Ctrl(crazyflie_msgs::GTC_Cmd_srv::Request &req)
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
    crazyflie_msgs::GTC_Cmd_srv srv;
    srv.request = req;
    CMD_Client.call(srv);


    // EXPERIMENT: 
    // BROADCAST CMD VALUES AS ROS MESSAGE
    // (SO CRAZYSWARM CAN PASS MSGS FROM BOTH DASHBOARD AND ENV FILE)
    crazyflie_msgs::GTC_Cmd cmd_msg;
    cmd_msg.cmd_type = req.cmd_type;
    cmd_msg.cmd_vals = req.cmd_vals;
    cmd_msg.cmd_flag = req.cmd_flag;
    cmd_msg.cmd_rx = req.cmd_rx;

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
    reward_vals = msg->reward_vals;

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

void CF_DataConverter::MainLoop()
{
    int loopRate = 1000;     // [Hz]
    ros::Rate rate(loopRate);


    
    while(ros::ok)
    {   

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


void CF_DataConverter::ConsoleLoop() // MAIN CONTROLLER LOOP
{
    set_escdelay(25);
    use_extended_names(TRUE);
    initscr();
    timeout(0); // Set getch() to non-blocking mode

    const int refresh_rate = 20; // 20 Hz
    const int delay_time_us = 1000000 / refresh_rate;

    while(true) {
        // Clear the screen buffer
        erase();

        mvprintw(0, 0,"t: %.4f V: %.3f",(Time-Time_start).toSec(),V_battery);
        mvprintw(1, 0,"DataType: %s \t Model: %s",DATA_TYPE.c_str(),MODEL_NAME.c_str());

        mvprintw(3, 0,"==== Flags ====");
        mvprintw(4, 0,"Motorstop:\t%u  Flip_flag:\t  %u  Pos Ctrl:\t    %u  Cam_Est:\t  %u",Motorstop_Flag, flip_flag, Pos_Ctrl_Flag,Camera_Sensor_Active);
        mvprintw(5, 0,"Traj Active:\t%u  Impact_flag:\t  %u  Vel Ctrl:\t    %u ",Traj_Active_Flag,impact_flag,Vel_Ctrl_Flag);
        mvprintw(6, 0,"Policy_armed:\t%u  Tumble Detect: %u  Moment_Flag:   %u ",Policy_Armed_Flag,Tumble_Detection,Moment_Flag);
        mvprintw(7, 0,"Sticky_flag:\t%u  Tumbled:\t  %u  Slowdown_type: %u",Sticky_Flag,Tumbled_Flag,SLOWDOWN_TYPE);
        
        mvprintw(9, 0,"==== System States ====");
        mvprintw(10, 0,"Pos [m]:\t % 8.3f  % 8.3f  % 8.3f",Pose.position.x,Pose.position.y,Pose.position.z);
        mvprintw(11, 0,"Vel [m/s]:\t % 8.3f  % 8.3f  % 8.3f",Twist.linear.x,Twist.linear.y,Twist.linear.z);
        mvprintw(12, 0,"Omega [rad/s]:\t % 8.3f  % 8.3f  % 8.3f",Twist.angular.x,Twist.angular.y,Twist.angular.z);
        mvprintw(13, 0,"Eul [deg]:\t % 8.3f  % 8.3f  % 8.3f",Eul.x,Eul.y,Eul.z);
        mvprintw(15, 0,"Vel [mag,phi,alph]: % 8.3f % 8.3f % 8.3f",Vel_mag,Phi,Alpha);


        mvprintw(17, 0,"Tau:     % 7.3f  \u03B8x:    % 7.3f  \u03B8y:    % 7.3f",Tau,Theta_x,Theta_y);
        mvprintw(18, 0,"Tau_est: % 7.3f  \u03B8x_est: % 7.3f  \u03B8y_est: % 7.3f",Tau_est,Theta_x_est,Theta_y_est);
        mvprintw(19, 0,"D_perp:  % 7.3f",D_perp);

        mvprintw(21, 0,"==== Policy: %s ====",POLICY_TYPE.c_str());
        if (strcmp(POLICY_TYPE.c_str(),"PARAM_OPTIM") == 0)
        {
            mvprintw(22, 0,"Tau_thr: % 7.3f \tMy: % 7.3f",Tau_thr,G1);
        }
        else if (strcmp(POLICY_TYPE.c_str(),"SVL_POLICY") == 0)
        {
            mvprintw(22, 0,"Policy_Flip: % 7.3f \tPolicy_Action: % 7.3f ",Policy_Flip,Policy_Action);
        }
        else if (strcmp(POLICY_TYPE.c_str(),"DEEP_RL") == 0)
        {
            mvprintw(22, 0,"Stuff: % 7.3f \tStuff: % 7.3f ",Policy_Flip,Policy_Action);
        }
        else if (strcmp(POLICY_TYPE.c_str(),"DEEP_RL_SB3") == 0)
        {
            mvprintw(22, 0,"SB3");
        }

        mvprintw(24,0,"==== Flip Trigger Values ====");
        mvprintw(25,0,"Tau_tr:     % 7.3f \tPolicy_Flip_tr:    % 7.3f ",Tau_tr,Policy_Flip_tr);
        mvprintw(26,0,"\u03B8x_tr:     % 7.3f \tPolicy_Action_tr:  % 7.3f ",Theta_x_tr,Policy_Action_tr);
        mvprintw(27,0,"D_perp_tr:  % 7.3f ",D_perp_tr);

        mvprintw(29,0,"==== Setpoints ====");
        mvprintw(30,0,"x_d: % 7.3f  % 7.3f  % 7.3f",x_d.x,x_d.y,x_d.z);
        mvprintw(31,0,"v_d: % 7.3f  % 7.3f  % 7.3f",v_d.x,v_d.y,v_d.z);
        mvprintw(32,0,"a_d: % 7.3f  % 7.3f  % 7.3f",a_d.x,a_d.y,a_d.z);

        mvprintw(34,0,"==== Controller Actions ====");
        mvprintw(35,0,"FM [N/N*mm]: % 7.3f  % 7.3f  % 7.3f  % 7.3f",FM[0],FM[1],FM[2],FM[3]);
        mvprintw(36,0,"Motor Thrusts [g]: % 7.3f  % 7.3f  % 7.3f  % 7.3f",MotorThrusts[0],MotorThrusts[1],MotorThrusts[2],MotorThrusts[3]);
        mvprintw(37,0,"MS_PWM: %u  %u  %u  %u",MS_PWM[0],MS_PWM[1],MS_PWM[2],MS_PWM[3]);


        mvprintw(39,0,"=== Controller Gains ====");
        mvprintw(40,0,"Kp_P: % 7.3f  % 7.3f  % 7.3f ",P_kp_xy,P_kp_xy,P_kp_z);
        mvprintw(40,37,"Kp_R: % 7.3f  % 7.3f  % 7.3f ",R_kp_xy,R_kp_xy,R_kp_z);

        mvprintw(41,0,"Kd_P: % 7.3f  % 7.3f  % 7.3f ",P_kd_xy,P_kd_xy,P_kd_z);
        mvprintw(41,37,"Kd_R: % 7.3f  % 7.3f  % 7.3f ",R_kd_xy,R_kd_xy,R_kd_z);

        mvprintw(42,0,"Ki_P: % 7.3f  % 7.3f  % 7.3f ",P_ki_xy,P_ki_xy,P_ki_z);
        mvprintw(42,37,"Ki_R: % 7.3f  % 7.3f  % 7.3f ",R_ki_xy,R_ki_xy,R_ki_z);
        mvprintw(43,0,"======\n");


        // Refresh the screen with the updated buffer
        refresh();

        // Sleep for the desired delay time
        usleep(delay_time_us);
    }

    // Clean up and close the ncurses library
    endwin();
}


int main(int argc, char** argv)
{
    setlocale(LC_CTYPE,"C-UTF-8");
    ros::init(argc,argv,"CF_DataConverter_Node");
    ros::NodeHandle nh;
    CF_DataConverter CF_DC = CF_DataConverter(&nh);
    ros::spin();
    return 0;
}