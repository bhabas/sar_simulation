/* 
This script subscribes to the ROS topics with the raw data from the crazyflie,
organizes it, and then republishes the data in an organized manner that 
is easy to use.
*/

// STANDARD INCLUDES
#include <stdio.h>
#include <iostream>
#include <boost/circular_buffer.hpp>
#include <math.h>       /* sqrt */
#include <thread>

// ROS INCLUDES
#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <gazebo_msgs/ContactsState.h>
#include <gazebo_msgs/SetPhysicsProperties.h>

// CUSTOM INCLUDES
#include "crazyflie_msgs/CF_StateData.h"
#include "crazyflie_msgs/CF_FlipData.h"
#include "crazyflie_msgs/CF_ImpactData.h"
#include "crazyflie_msgs/CF_MiscData.h"

#include "crazyflie_msgs/CtrlData.h"
#include "crazyflie_msgs/CtrlDebug.h"
#include "crazyflie_msgs/RLCmd.h"
#include "crazyflie_msgs/PadConnect.h"

class CF_DataConverter
{
    public:
        // CONSTRUCTOR TO START PUBLISHERS AND SUBSCRIBERS (Similar to Python's __init__())
        CF_DataConverter(ros::NodeHandle* nh)
        {

            // INITIALIZE SUBSCRIBERS
            CTRL_Data_Sub = nh->subscribe("/CTRL/data", 1, &CF_DataConverter::CtrlData_Callback, this, ros::TransportHints().tcpNoDelay());
            CTRL_Debug_Sub = nh->subscribe("/CTRL/debug", 1, &CF_DataConverter::CtrlDebug_Callback, this, ros::TransportHints().tcpNoDelay());
            RL_CMD_Sub = nh->subscribe("/RL/cmd",5,&CF_DataConverter::RL_CMD_Callback,this,ros::TransportHints().tcpNoDelay());
            Surface_FT_Sub = nh->subscribe("/ENV/Surface_FT_sensor",5,&CF_DataConverter::SurfaceFT_Sensor_Callback,this,ros::TransportHints().tcpNoDelay());
            Surface_Contact_Sub = nh->subscribe("/ENV/BodyContact",5,&CF_DataConverter::Surface_Contact_Callback,this,ros::TransportHints().tcpNoDelay());
            PadConnect_Sub = nh->subscribe("/ENV/Pad_Connections",5,&CF_DataConverter::Pad_Connections_Callback,this,ros::TransportHints().tcpNoDelay());

            // INITIALIZE MAIN PUBLISHERS
            StateData_Pub = nh->advertise<crazyflie_msgs::CF_StateData>("/CF_DC/StateData",1);
            MiscData_Pub =  nh->advertise<crazyflie_msgs::CF_MiscData>("/CF_DC/MiscData",1);
            FlipData_Pub =  nh->advertise<crazyflie_msgs::CF_FlipData>("/CF_DC/FlipData",1);
            ImpactData_Pub = nh->advertise<crazyflie_msgs::CF_ImpactData>("/CF_DC/ImpactData",1);   

            // GAZEBO SERVICES
            GZ_SimSpeed_Client = nh->serviceClient<gazebo_msgs::SetPhysicsProperties>("/gazebo/set_physics_properties");

            

            CF_DataConverter::LoadParams();

            CF_DataConverter::adjustSimSpeed(SIM_SPEED);
            BodyCollision_str = MODEL_NAME + "::crazyflie_ModelBase::crazyflie_body::body_collision";

            controllerThread = std::thread(&CF_DataConverter::MainLoop, this);


        }


        // FUNCTION PRIMITIVES
        void CtrlData_Callback(const crazyflie_msgs::CtrlData &ctrl_msg);
        void CtrlDebug_Callback(const crazyflie_msgs::CtrlDebug &ctrl_msg);

        void RL_CMD_Callback(const crazyflie_msgs::RLCmd::ConstPtr &msg);
        void SurfaceFT_Sensor_Callback(const geometry_msgs::WrenchStamped::ConstPtr &msg);
        void Surface_Contact_Callback(const gazebo_msgs::ContactsState &msg);
        void Pad_Connections_Callback(const crazyflie_msgs::PadConnect &msg);

        void Publish_StateData();
        void Publish_FlipData();
        void Publish_ImpactData();
        void Publish_MiscData();

        void MainLoop();
        void LoadParams();
        void consoleOuput();
        void checkSlowdown();
        void adjustSimSpeed(float speed_mult);
        void decompressXY(uint32_t xy, float xy_arr[]);
        void quat2euler(float quat[], float eul[]);

    private:

        // SUBSCRIBERS
        ros::Subscriber CTRL_Data_Sub;
        ros::Subscriber CTRL_Debug_Sub;
        ros::Subscriber RL_CMD_Sub;
        ros::Subscriber Surface_FT_Sub;
        ros::Subscriber Surface_Contact_Sub;
        ros::Subscriber PadConnect_Sub;

        // PUBLISHERS
        ros::Publisher StateData_Pub;
        ros::Publisher FlipData_Pub;
        ros::Publisher ImpactData_Pub;
        ros::Publisher MiscData_Pub;

        // SERVICES
        ros::ServiceClient GZ_SimSpeed_Client;

        // MESSAGES
        crazyflie_msgs::CF_StateData StateData_msg;
        crazyflie_msgs::CF_FlipData FlipData_msg;
        crazyflie_msgs::CF_ImpactData ImpactData_msg;
        crazyflie_msgs::CF_MiscData MiscData_msg;

        std::thread controllerThread;
        std::string BodyCollision_str;
        uint32_t tick = 1;
        
        std::string MODEL_NAME;
        float H_CEILING = 2.10;

        int SLOWDOWN_TYPE = 0;
        bool LANDING_SLOWDOWN_FLAG;
        float SIM_SPEED; 
        float SIM_SLOWDOWN_SPEED;
        int POLICY_TYPE = 0;



        // ===================
        //     FLIGHT DATA
        // ===================

        ros::Time Time;

        geometry_msgs::Pose Pose;
        geometry_msgs::Twist Twist;
        geometry_msgs::Vector3 Eul;

        double Tau;
        double OFx;
        double OFy;
        double RREV;
        double D_ceil;

        boost::array<double, 4> FM;
        boost::array<uint16_t, 4> MS_PWM;

        double RREV_thr;
        double G1;

        double NN_flip;
        double NN_policy;

        geometry_msgs::Vector3 x_d;
        geometry_msgs::Vector3 v_d;
        geometry_msgs::Vector3 a_d;

        // ==================
        //     FLIP DATA
        // ==================

        bool flip_flag = false;
        bool OnceFlag_flip = false;

        ros::Time Time_tr;

        geometry_msgs::Pose Pose_tr;
        geometry_msgs::Twist Twist_tr;
        geometry_msgs::Vector3 Eul_tr;


        double Tau_tr;
        double OFx_tr;
        double OFy_tr;
        double RREV_tr;
        double D_ceil_tr;

        boost::array<double, 4> FM_tr;

        double NN_tr_flip;
        double NN_tr_policy;


        // ===================
        //     IMPACT DATA
        // ===================

        bool impact_flag = false;
        bool BodyContact_flag = false;
        bool OnceFlag_impact = false;
        double impact_thr = 0.1; // Impact threshold [N]

        ros::Time Time_impact;
        geometry_msgs::Vector3 Force_impact;
        geometry_msgs::Pose Pose_impact;
        geometry_msgs::Twist Twist_impact;
        geometry_msgs::Vector3 Eul_impact;


        double impact_force_x = 0.0; // Max impact force in X-direction [N]
        double impact_force_y = 0.0; // Max impact force in Y-direction [N]
        double impact_force_z = 0.0; // Max impact force in Z-direction [N]
        double impact_force_resultant = 0.0; // Current impact force magnitude

        // CIRCULAR BUFFERES TO LAG IMPACT STATE DATA (WE WANT STATE DATA THE INSTANT BEFORE IMPACT)
        boost::circular_buffer<geometry_msgs::Pose> Pose_impact_buff {5};
        boost::circular_buffer<geometry_msgs::Twist> Twist_impact_buff {5};

        // ==================
        //     MISC DATA
        // ==================

        double V_battery = 4.0;

        uint8_t Pad1_Contact = 0;
        uint8_t Pad2_Contact = 0;
        uint8_t Pad3_Contact = 0;
        uint8_t Pad4_Contact = 0;

        uint8_t Pad_Connections = 0;

        // ====================
        //     DEBUG VALUES
        // ====================

        bool Motorstop_Flag = false;
        bool Pos_Ctrl_Flag = false;
        bool Vel_Ctrl_Flag = false;
        bool Traj_Active_Flag = false;
        bool Tumble_Detection = false;
        bool Tumbled_Flag = false;
        bool Moment_Flag = false;
        bool Policy_Armed_Flag = false;
        bool Sticky_Flag = false;


        float P_kp_xy;
        float P_kd_xy;
        float P_ki_xy;
        float P_kp_z;
        float P_kd_z;
        float P_ki_z;
        float R_kp_xy;
        float R_kd_xy;
        float R_ki_xy;     
        float R_kp_z;
        float R_kd_z;
        float R_ki_z;

};

void CF_DataConverter::LoadParams()
{
    ros::param::get("/MODEL_NAME",MODEL_NAME);
    ros::param::get("/CEILING_HEIGHT",H_CEILING);
    // ros::param::get("/CF_MASS",_CF_MASS);
    ros::param::get("/POLICY_TYPE",POLICY_TYPE);

    // // DEBUG SETTINGS
    ros::param::get("/SIM_SPEED",SIM_SPEED);
    ros::param::get("/SIM_SLOWDOWN_SPEED",SIM_SLOWDOWN_SPEED);
    ros::param::get("/LANDING_SLOWDOWN_FLAG",LANDING_SLOWDOWN_FLAG);

    ros::param::get("P_kp_xy",P_kp_xy);
    ros::param::get("P_kd_xy",P_kd_xy);
    ros::param::get("P_ki_xy",P_ki_xy);

    ros::param::get("P_kp_z",P_kp_z);
    ros::param::get("P_kd_z",P_kd_z);
    ros::param::get("P_ki_z",P_ki_z);

    ros::param::get("R_kp_xy",R_kp_xy);
    ros::param::get("R_kd_xy",R_kd_xy);
    ros::param::get("R_ki_xy",R_ki_xy);
    
    ros::param::get("R_kp_z",R_kp_z);
    ros::param::get("R_kd_z",R_kd_z);
    ros::param::get("R_ki_z",R_ki_z);

}

void CF_DataConverter::consoleOuput()
{
    system("clear");
    printf("t: %.4f \tCmd: \n",Time.toSec());
    printf("Model: %s\n",MODEL_NAME.c_str());
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
    printf("RREV_thr: %.3f \tG1: %.3f \tG2: %.3f\n",RREV_thr,G1,0.0);
    printf("\n");

    printf("NN_Outputs: \n");
    printf("NN_Flip:  %.3f \tNN_Policy: %.3f \n",NN_flip,NN_policy);
    printf("\n");

    printf("==== Flip Trigger Values ====\n");
    printf("RREV_tr:    %.3f \tNN_tr_Flip:    %.3f \n",RREV_tr,NN_tr_flip);
    printf("OFy_tr:     %.3f \tNN_tr_Policy:  %.3f \n",OFy_tr,NN_tr_policy);
    printf("D_ceil_tr:  %.3f \n",D_ceil_tr);
    printf("\n");

    printf("==== Controller Actions ====\n");
    printf("FM [N/N*mm]: %.3f  %.3f  %.3f  %.3f\n",FM[0],FM[1],FM[2],FM[3]);
    printf("MS_PWM: %u  %u  %u  %u\n",MS_PWM[0],MS_PWM[1],MS_PWM[2],MS_PWM[3]);
    printf("\n");


    printf("=== Parameters ====\n");
    printf("Kp_P: %.3f  %.3f  %.3f \t",P_kp_xy,P_kp_xy,P_kp_z);
    printf("Kp_R: %.3f  %.3f  %.3f \n",R_kd_xy,R_kd_xy,R_kd_z);
    printf("Kd_P: %.3f  %.3f  %.3f \t",P_kp_xy,P_kp_xy,P_kp_z);
    printf("Kd_R: %.3f  %.3f  %.3f \n",R_kd_xy,R_kd_xy,R_kd_z);
    printf("Ki_P: %.3f  %.3f  %.3f \t",P_ki_xy,P_ki_xy,P_ki_z);
    printf("Ki_R: %.3f  %.3f  %.3f \n",R_ki_xy,R_ki_xy,R_ki_z);
    printf("======\n");
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

