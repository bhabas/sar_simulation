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
#include <geometry_msgs/Point.h>

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
#include "crazyflie_msgs/RLData.h"
#include "crazyflie_msgs/PadConnect.h"

#include "crazyflie_msgs/activateSticky.h"
#include "crazyflie_msgs/loggingCMD.h"
#include "crazyflie_msgs/GenericLogData.h"

#include "quatcompress.h"


#define formatBool(b) ((b) ? "True" : "False")
#define RATE_DO_EXECUTE(RATE_HZ, TICK) ((TICK % (100 / RATE_HZ)) == 0)

class CF_DataConverter
{
    public:
        // CONSTRUCTOR TO START PUBLISHERS AND SUBSCRIBERS (Similar to Python's __init__())
        CF_DataConverter(ros::NodeHandle* nh)
        {

            // INITIALIZE SUBSCRIBERS
            CTRL_Data_Sub = nh->subscribe("/CTRL/data", 1, &CF_DataConverter::CtrlData_Callback, this, ros::TransportHints().tcpNoDelay());
            CTRL_Debug_Sub = nh->subscribe("/CTRL/debug", 1, &CF_DataConverter::CtrlDebug_Callback, this, ros::TransportHints().tcpNoDelay());
            RL_Data_Sub = nh->subscribe("/RL/data",5,&CF_DataConverter::RL_Data_Callback,this,ros::TransportHints().tcpNoDelay());
            Surface_FT_Sub = nh->subscribe("/ENV/Surface_FT_sensor",5,&CF_DataConverter::SurfaceFT_Sensor_Callback,this,ros::TransportHints().tcpNoDelay());
            Surface_Contact_Sub = nh->subscribe("/ENV/BodyContact",5,&CF_DataConverter::Surface_Contact_Callback,this,ros::TransportHints().tcpNoDelay());
            PadConnect_Sub = nh->subscribe("/ENV/Pad_Connections",5,&CF_DataConverter::Pad_Connections_Callback,this,ros::TransportHints().tcpNoDelay());

            log1_Sub = nh->subscribe("/cf1/log1", 1, &CF_DataConverter::log1_Callback, this, ros::TransportHints().tcpNoDelay());
            log2_Sub = nh->subscribe("/cf1/log2", 1, &CF_DataConverter::log2_Callback, this, ros::TransportHints().tcpNoDelay());
            log3_Sub = nh->subscribe("/cf1/log3", 1, &CF_DataConverter::log3_Callback, this, ros::TransportHints().tcpNoDelay());
            log4_Sub = nh->subscribe("/cf1/log4", 1, &CF_DataConverter::log4_Callback, this, ros::TransportHints().tcpNoDelay());
            log5_Sub = nh->subscribe("/cf1/log5", 1, &CF_DataConverter::log5_Callback, this, ros::TransportHints().tcpNoDelay());

            // INITIALIZE MAIN PUBLISHERS
            StateData_Pub = nh->advertise<crazyflie_msgs::CF_StateData>("/CF_DC/StateData",1);
            MiscData_Pub =  nh->advertise<crazyflie_msgs::CF_MiscData>("/CF_DC/MiscData",1);
            FlipData_Pub =  nh->advertise<crazyflie_msgs::CF_FlipData>("/CF_DC/FlipData",1);
            ImpactData_Pub = nh->advertise<crazyflie_msgs::CF_ImpactData>("/CF_DC/ImpactData",1);   

            // GAZEBO SERVICES
            GZ_SimSpeed_Client = nh->serviceClient<gazebo_msgs::SetPhysicsProperties>("/gazebo/set_physics_properties");
            Logging_Service = nh->advertiseService("/CF_DC/DataLogging", &CF_DataConverter::DataLogging_Callback, this);

            CMD_Service_CF_DC = nh->advertiseService("/CF_DC/Cmd_CF_DC",&CF_DataConverter::CMD_CF_DC_Callback,this);
            CMD_Service_Dashboard = nh->advertiseService("/CF_DC/Cmd_Dashboard",&CF_DataConverter::CMD_Dashboard_Callback,this);
            CMD_Client = nh->serviceClient<crazyflie_msgs::RLCmd>("/CTRL/Cmd_ctrl");
            

            
            

            CF_DataConverter::LoadParams();
            CF_DataConverter::adjustSimSpeed(SIM_SPEED);
            Time_start = ros::Time::now();


            BodyCollision_str = MODEL_NAME + "::crazyflie_BaseModel::crazyflie_body::body_collision";

            CF_DCThread = std::thread(&CF_DataConverter::MainLoop, this);


        }




        // SIMULATION DATA CALLBACKS
        void CtrlData_Callback(const crazyflie_msgs::CtrlData &ctrl_msg);
        void CtrlDebug_Callback(const crazyflie_msgs::CtrlDebug &ctrl_msg);

        void RL_Data_Callback(const crazyflie_msgs::RLData::ConstPtr &msg);
        void SurfaceFT_Sensor_Callback(const geometry_msgs::WrenchStamped::ConstPtr &msg);
        void Surface_Contact_Callback(const gazebo_msgs::ContactsState &msg);
        void Pad_Connections_Callback(const crazyflie_msgs::PadConnect &msg);


        bool CMD_CF_DC_Callback(crazyflie_msgs::RLCmd::Request &req, crazyflie_msgs::RLCmd::Response &res);
        bool CMD_Dashboard_Callback(crazyflie_msgs::RLCmd::Request &req, crazyflie_msgs::RLCmd::Response &res);
        bool Send_Cmd2Ctrl(crazyflie_msgs::RLCmd::Request &req);



        // EXPERIMENT DATA CALLBACKS
        void log1_Callback(const crazyflie_msgs::GenericLogData::ConstPtr &log1_msg);
        void log2_Callback(const crazyflie_msgs::GenericLogData::ConstPtr &log2_msg);
        void log3_Callback(const crazyflie_msgs::GenericLogData::ConstPtr &log3_msg);
        void log4_Callback(const crazyflie_msgs::GenericLogData::ConstPtr &log4_msg);
        void log5_Callback(const crazyflie_msgs::GenericLogData::ConstPtr &log5_msg);

        // ORGANIZED DATA PUBLISHERS
        void Publish_StateData();
        void Publish_FlipData();
        void Publish_ImpactData();
        void Publish_MiscData();

        // LOGGING FUNCTIONS
        bool DataLogging_Callback(crazyflie_msgs::loggingCMD::Request &req, crazyflie_msgs::loggingCMD::Response &res);
        void create_CSV();
        void append_CSV_states();
        void append_CSV_misc();
        void append_CSV_flip();
        void append_CSV_impact();
        void append_CSV_blank();

        // SIMULATION FUNCTIONS
        void activateStickyFeet();
        void checkSlowdown();
        void adjustSimSpeed(float speed_mult);

        
        void MainLoop();
        void LoadParams();
        void consoleOuput();
        void decompressXY(uint32_t xy, float xy_arr[]);
        void quat2euler(float quat[], float eul[]);

    private:

        // SUBSCRIBERSCMD_Service_CF_DC
        ros::Subscriber CTRL_Data_Sub;
        ros::Subscriber CTRL_Debug_Sub;
        ros::Subscriber RL_CMD_Sub;
        ros::Subscriber RL_Data_Sub;
        ros::Subscriber Surface_FT_Sub;
        ros::Subscriber Surface_Contact_Sub;
        ros::Subscriber PadConnect_Sub;

        // SUBSCRIBERS
        ros::Subscriber log1_Sub;
        ros::Subscriber log2_Sub;
        ros::Subscriber log3_Sub;
        ros::Subscriber log4_Sub;
        ros::Subscriber log5_Sub;

        // PUBLISHERS
        ros::Publisher StateData_Pub;
        ros::Publisher FlipData_Pub;
        ros::Publisher ImpactData_Pub;
        ros::Publisher MiscData_Pub;

        // SERVICES
        ros::ServiceClient GZ_SimSpeed_Client;
        ros::ServiceServer Logging_Service;

        ros::ServiceServer CMD_Service_CF_DC;
        ros::ServiceServer CMD_Service_Dashboard;
        ros::ServiceClient CMD_Client;

        // MESSAGES
        crazyflie_msgs::CF_StateData StateData_msg;
        crazyflie_msgs::CF_FlipData FlipData_msg;
        crazyflie_msgs::CF_ImpactData ImpactData_msg;
        crazyflie_msgs::CF_MiscData MiscData_msg;

        std::thread CF_DCThread;
        uint32_t tick = 1;      // Tick for each loop iteration
        ros::Time Time_start;   // Initial time in UNIX notation

        // LOGGING VALS
        FILE* fPtr; // File Pointer to logging file
        bool Logging_Flag = false;
        std::string error_string;

        
        // ===================
        //     ROS PARAMS
        // ===================

        std::string MODEL_NAME;
        std::string DATA_TYPE;

        // DEFAULT INERTIA VALUES FOR BASE CRAZYFLIE
        float CF_MASS = 34.4e3; // [kg]
        float Ixx = 15.83e-6f;  // [kg*m^2]
        float Iyy = 17.00e-6f;  // [kg*m^2]
        float Izz = 31.19e-6f;  // [kg*m^2]

        int SLOWDOWN_TYPE = 0;
        bool LANDING_SLOWDOWN_FLAG;
        float SIM_SPEED; 
        float SIM_SLOWDOWN_SPEED;
        int LOGGING_RATE = 20;
        int CONSOLE_RATE = 50;
        int POLICY_TYPE = 0;
        
        float P_kp_xy,P_kd_xy,P_ki_xy;
        float P_kp_z,P_kd_z,P_ki_z;
        float R_kp_xy,R_kd_xy,R_ki_xy;     
        float R_kp_z,R_kd_z,R_ki_z;



        // ===================
        //     FLIGHT DATA
        // ===================

        ros::Time Time;

        geometry_msgs::Pose Pose;
        geometry_msgs::Twist Twist;
        geometry_msgs::Vector3 Eul;

        double Tau = 0.0;
        double OFx = 0.0;
        double OFy = 0.0;
        double D_ceil = 0.0;

        double Tau_est = 0.0;
        double OFx_est = 0.0;
        double OFy_est = 0.0;

        boost::array<double,4> FM{0,0,0,0};
        boost::array<double,4> MotorThrusts{0,0,0,0};
        boost::array<uint16_t,4> MS_PWM{0,0,0,0};

        double Tau_thr = 0.0;
        double G1 = 0.0;

        double NN_flip = 0.0;
        double NN_policy = 0.0;

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


        double Tau_tr = 0.0;
        double OFx_tr = 0.0;
        double OFy_tr = 0.0;
        double D_ceil_tr = 0.0;

        boost::array<double,4> FM_tr{0,0,0,0};

        double NN_tr_flip = 0.0;
        double NN_tr_policy = 0.0;


        // ===================
        //     IMPACT DATA
        // ===================

        bool impact_flag = false;
        bool BodyContact_flag = false;
        bool OnceFlag_impact = false;
        double impact_thr = 0.1;        // Impact threshold [N]
        std::string BodyCollision_str;  // String of Body Name


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
        bool Camera_Sensor_Active = false;
        bool Sticky_Flag = false;


        // ===================
        //     RL DATA
        // ===================
        uint8_t k_ep = 0;
        uint8_t k_run = 0;
        uint8_t n_rollouts = 8;

        boost::array<double,2> mu{0,0};
        boost::array<float,2> sigma{0,0};
        boost::array<float,3> policy{0,0,0};

        float reward = 0.0;
        boost::array<double,3> reward_inputs{0,0,0};

        boost::array<float,3> vel_d{0,0,0};

        bool runComplete_flag = false;

};

void CF_DataConverter::log1_Callback(const crazyflie_msgs::GenericLogData::ConstPtr &log1_msg)
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
    
    OFx = OF_xy_arr[0];
    OFy = OF_xy_arr[1];
    Tau = log1_msg->values[7]*1e-3;

}

void CF_DataConverter::log2_Callback(const crazyflie_msgs::GenericLogData::ConstPtr &log2_msg)
{
    // ANGULAR VELOCITY (Z)
    Twist.angular.z = log2_msg->values[0]*1e-3;

    // CEILING DISTANCE
    D_ceil = log2_msg->values[1]*1e-3;

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
        round(MS_PWM12[0]*2.0e3),
        round(MS_PWM12[1]*2.0e3), 
        round(MS_PWM34[0]*2.0e3),
        round(MS_PWM34[1]*2.0e3)
    };
    
    // NEURAL NETWORK VALUES
    float NN_FP[2];
    decompressXY(log2_msg->values[6],NN_FP);
    NN_flip = NN_FP[0];
    NN_policy = NN_FP[1];

    // OTHER MISC INFO
    flip_flag = log2_msg->values[7];
    if(flip_flag == true && OnceFlag_flip == false)
    {
        Time_tr = ros::Time::now();
        OnceFlag_flip = true;
    }

    V_battery = 3.5 + (log2_msg->values[8]/256)*(4.2-3.5);


}

void CF_DataConverter::log3_Callback(const crazyflie_msgs::GenericLogData::ConstPtr &log3_msg)
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

void CF_DataConverter::log4_Callback(const crazyflie_msgs::GenericLogData::ConstPtr &log4_msg)
{
    // FLIP TRIGGER - POSITION
    Pose_tr.position.x = NAN;
    Pose_tr.position.y = NAN;
    Pose_tr.position.z = log4_msg->values[0]*1e-3;

    // FLIP TRIGGER - CEILING DISTANCE
    D_ceil_tr = log4_msg->values[1]*1e-3;

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
    
    OFx_tr = OF_xy_arr[0];
    OFy_tr = OF_xy_arr[1];
    Tau_tr = log4_msg->values[7]*1e-3;

}

void CF_DataConverter::log5_Callback(const crazyflie_msgs::GenericLogData::ConstPtr &log5_msg)
{
    Motorstop_Flag = log5_msg->values[0];
    Pos_Ctrl_Flag = log5_msg->values[1];
    Vel_Ctrl_Flag = log5_msg->values[2];
    Traj_Active_Flag = log5_msg->values[3];
    Tumbled_Flag = log5_msg->values[4];
    Moment_Flag = log5_msg->values[5];
    Policy_Armed_Flag =log5_msg->values[6];

}

void CF_DataConverter::LoadParams()
{

    // COLLECT MODEL PARAMETERS
    ros::param::get("/MODEL_NAME",MODEL_NAME);
    ros::param::get("/CF_MASS",CF_MASS);
    ros::param::get("/Ixx",Ixx);
    ros::param::get("/Iyy",Iyy);
    ros::param::get("/Izz",Izz);
    ros::param::get("/POLICY_TYPE",POLICY_TYPE);

    // DEBUG SETTINGS
    ros::param::get("/DATA_TYPE",DATA_TYPE);
    ros::param::get("/SIM_SPEED",SIM_SPEED);
    ros::param::get("/SIM_SLOWDOWN_SPEED",SIM_SLOWDOWN_SPEED);
    ros::param::get("/LANDING_SLOWDOWN_FLAG",LANDING_SLOWDOWN_FLAG);
    ros::param::get("/LOGGING_RATE",LOGGING_RATE);
    ros::param::get("/CONSOLE_RATE",CONSOLE_RATE);

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

    if(DATA_TYPE.compare("SIM") == 0)
    {
        ros::param::set("/use_sim_time",true);
    }
    else
    {
        ros::param::set("/use_sim_time",false);
    }

}



// DECOMPRESS COMBINED PAIR OF VALUES FROM CF MESSAGE INTO THEIR RESPECTIVE FLOAT VALUES
void CF_DataConverter::decompressXY(uint32_t xy, float xy_arr[])
{
    uint16_t xd = ((uint32_t)xy >> 16) & 0xFFFF;    // Shift out y-value bits
    xy_arr[0] = ((float)xd - 32767.0f)*1e-3;        // Convert to normal value

    uint16_t yd = (uint32_t)xy & 0xFFFF;            // Save only y-value bits
    xy_arr[1] = ((float)yd - 32767.0f)*1e-3;

}



// =========================
//     LOGGING FUNCTIONS
// =========================

void CF_DataConverter::create_CSV()
{
    fprintf(fPtr,"k_ep,k_run,");
    fprintf(fPtr,"t,");
    fprintf(fPtr,"NN_flip,NN_policy,");
    fprintf(fPtr,"mu,sigma,policy,");

    // INTERNAL STATE ESTIMATES (CF)
    fprintf(fPtr,"x,y,z,");
    fprintf(fPtr,"vx,vy,vz,");
    fprintf(fPtr,"qx,qy,qz,qw,");
    fprintf(fPtr,"wx,wy,wz,");
    fprintf(fPtr,"eul_x,eul_y,eul_z,");

    // MISC RL LABELS
    fprintf(fPtr,"flip_flag,impact_flag,");

    //  MISC INTERNAL STATE ESTIMATES
    fprintf(fPtr,"Tau_est,OF_x_est,OF_y_est,");
    fprintf(fPtr,"Tau,OF_x,OF_y,d_ceil,");
    fprintf(fPtr,"F_thrust,Mx,My,Mz,");
    fprintf(fPtr,"M1_thrust,M2_thrust,M3_thrust,M4_thrust,");
    fprintf(fPtr,"M1_pwm,M2_pwm,M3_pwm,M4_pwm,");


    // SETPOINT VALUES
    fprintf(fPtr,"x_d.x,x_d.y,x_d.z,");
    fprintf(fPtr,"v_d.x,v_d.y,v_d.z,");
    fprintf(fPtr,"a_d.x,a_d.y,a_d.z,");

    // MISC VALUES
    fprintf(fPtr,"Volts,Error");

    fprintf(fPtr,"\n");
    fflush(fPtr);

}

void CF_DataConverter::append_CSV_states()
{
    fprintf(fPtr,"%u,%u,",k_ep,k_run);              // k_ep,k_run
    fprintf(fPtr,"%.3f,",(Time-Time_start).toSec());             // t
    fprintf(fPtr,"%.3f,%.3f,",NN_flip,NN_policy);   // NN_flip,NN_policy
    fprintf(fPtr,"--,--,--,");                      // mu,sigma,policy

    // // INTERNAL STATE ESTIMATES (CF)
    fprintf(fPtr,"%.3f,%.3f,%.3f,",Pose.position.x,Pose.position.y,Pose.position.z);    // x,y,z
    fprintf(fPtr,"%.3f,%.3f,%.3f,",Twist.linear.x,Twist.linear.y,Twist.linear.z);       // vx,vy,vz
    fprintf(fPtr,"%.3f,%.3f,%.3f,%.3f,",Pose.orientation.x,Pose.orientation.y,Pose.orientation.z,Pose.orientation.w); // qx,qy,qz,qw
    fprintf(fPtr,"%.3f,%.3f,%.3f,",Twist.angular.x,Twist.angular.y,Twist.angular.z);    // wx,wy,wz
    fprintf(fPtr,"%.3f,%.3f,%.3f,",Eul.x,Eul.y,Eul.z);                                  // eul_x,eul_y,eul_z


    // MISC RL LABELS
    fprintf(fPtr,"%s,%s,",formatBool(flip_flag),formatBool(impact_flag));   // flip_flag,impact_flag

    // MISC INTERNAL STATE ESTIMATES
    fprintf(fPtr,"%.3f,%.3f,%.3f,",Tau_est,OFx_est,OFy_est);      // Tau,OF_x,OF_y,d_ceil
    fprintf(fPtr,"%.3f,%.3f,%.3f,%.3f,",Tau,OFx,OFy,D_ceil);      // Tau,OF_x,OF_y,d_ceil
    fprintf(fPtr,"%.3f,%.3f,%.3f,%.3f,",FM[0],FM[1],FM[2],FM[3]);           // F_thrust,Mx,My,Mz
    fprintf(fPtr,"%.3f,%.3f,%.3f,%.3f,",MotorThrusts[0],MotorThrusts[1],MotorThrusts[2],MotorThrusts[3]); // M1_thrust,M2_thrust,M3_thrust,M4_thrust
    fprintf(fPtr,"%u,%u,%u,%u,",MS_PWM[0],MS_PWM[1],MS_PWM[2],MS_PWM[3]);   // M1_pwm,M2_pwm,M3_pwm,M4_pwm


    // SETPOINT VALUES
    fprintf(fPtr,"%.3f,%.3f,%.3f,",x_d.x,x_d.y,x_d.z);  // x_d.x,x_d.y,x_d.z
    fprintf(fPtr,"%.3f,%.3f,%.3f,",v_d.x,v_d.y,v_d.z);  // v_d.x,v_d.y,v_d.z
    fprintf(fPtr,"%.3f,%.3f,%.3f,",a_d.x,a_d.y,a_d.z);  // a_d.x,a_d.y,a_d.z


    // MISC VALUES
    fprintf(fPtr,"%.3f,%s",V_battery,"--"); // Volts,Error
    fprintf(fPtr,"\n");
    fflush(fPtr);

}

void CF_DataConverter::append_CSV_misc()
{
    fprintf(fPtr,"%u,%u,",k_ep,k_run);  // k_ep,k_run
    fprintf(fPtr,"--,");                // --
    fprintf(fPtr,"%u,--,",n_rollouts);  // n_rollouts,--
    fprintf(fPtr,"[%.3f %.3f],[%.3f %.3f],[%.3f %.3f],",mu[0],mu[1],sigma[0],sigma[1],policy[0],policy[1]); // mu,sigma,policy

    // // INTERNAL STATE ESTIMATES (CF)
    fprintf(fPtr,"--,--,--,");      // x,y,z
    fprintf(fPtr,"%.3f,%.3f,%.3f,",vel_d[0],vel_d[1],vel_d[2]); // vel_d.x,vel_d.y,vel_d.z
    fprintf(fPtr,"--,--,--,--,");   // qx,qy,qz,qw
    fprintf(fPtr,"--,--,--,");      // wx,wy,wz
    fprintf(fPtr,"--,--,--,");      // eul_x,eul_y,eul_z




    // MISC RL LABELS
    fprintf(fPtr,"%.2f,[%.3f %.3f %.3f],",reward,reward_inputs[0],reward_inputs[1],reward_inputs[2]);

    // MISC INTERNAL STATE ESTIMATES
    fprintf(fPtr,"--,--,--,"); // Tau_est,OF_x_est,OF_y_est
    fprintf(fPtr,"--,--,--,--,"); // Tau,OF_x,OF_y,d_ceil
    fprintf(fPtr,"--,--,--,--,"); // F_thrust,Mx,My,Mz
    fprintf(fPtr,"--,--,--,--,"); // M1_thrust,M2_thrust,M3_thrust,M4_thrust
    fprintf(fPtr,"--,--,--,--,"); // M1_pwm,M2_pwm,M3_pwm,M4_pwm
    

    // SETPOINT VALUES
    fprintf(fPtr,"--,--,--,");  // x_d.x,x_d.y,x_d.z
    fprintf(fPtr,"--,--,--,");  // v_d.x,v_d.y,v_d.z
    fprintf(fPtr,"--,--,--,");  // a_d.x,a_d.y,a_d.z



    // MISC VALUES
    fprintf(fPtr,"--,%s",error_string.c_str()); // Volts,Error
    fprintf(fPtr,"\n");
    fflush(fPtr);

}

void CF_DataConverter::append_CSV_flip()
{
    fprintf(fPtr,"%u,%u,",k_ep,k_run);
    fprintf(fPtr,"%.3f,",(Time_tr-Time_start).toSec());
    fprintf(fPtr,"%.3f,%.3f,",NN_tr_flip,NN_tr_policy);
    fprintf(fPtr,"--,--,--,");

    // // INTERNAL STATE ESTIMATES (CF)
    fprintf(fPtr,"%.3f,%.3f,%.3f,",Pose_tr.position.x,Pose_tr.position.y,Pose_tr.position.z);
    fprintf(fPtr,"%.3f,%.3f,%.3f,",Twist_tr.linear.x,Twist_tr.linear.y,Twist_tr.linear.z);
    fprintf(fPtr,"%.3f,%.3f,%.3f,%.3f,",Pose_tr.orientation.x,Pose_tr.orientation.y,Pose_tr.orientation.z,Pose_tr.orientation.w);
    fprintf(fPtr,"%.3f,%.3f,%.3f,",Twist_tr.angular.x,Twist_tr.angular.y,Twist_tr.angular.z);
    fprintf(fPtr,"%.3f,%.3f,%.3f,",Eul_tr.x,Eul_tr.y,Eul_tr.z);


    // MISC RL LABELS
    fprintf(fPtr,"%s,--,",formatBool(flip_flag));

    // MISC INTERNAL STATE ESTIMATES
    fprintf(fPtr,"--,--,--,"); // Tau_est,OF_x_est,OF_y_est
    fprintf(fPtr,"%.3f,%.3f,%.3f,%.3f,",Tau_tr,OFx_tr,OFy_tr,D_ceil_tr);
    fprintf(fPtr,"%.3f,%.3f,%.3f,%.3f,",FM_tr[0],FM_tr[1],FM_tr[2],FM_tr[3]);
    fprintf(fPtr,"--,--,--,--,");
    fprintf(fPtr,"--,--,--,--,");


    // SETPOINT VALUES
    fprintf(fPtr,"--,--,--,");
    fprintf(fPtr,"--,--,--,");
    fprintf(fPtr,"--,--,--,");


    // MISC VALUES
    fprintf(fPtr,"--,%s","Flip Data");
    fprintf(fPtr,"\n");
    fflush(fPtr);

}

void CF_DataConverter::append_CSV_impact()
{
    fprintf(fPtr,"%u,%u,",k_ep,k_run);
    fprintf(fPtr,"%.3f,",(Time_impact-Time_start).toSec());
    fprintf(fPtr,"--,--,");
    fprintf(fPtr,"--,--,--,");

    // // INTERNAL STATE ESTIMATES (CF)
    fprintf(fPtr,"%.3f,%.3f,%.3f,",Pose_impact.position.x,Pose_impact.position.y,Pose_impact.position.z);
    fprintf(fPtr,"%.3f,%.3f,%.3f,",Twist_impact.linear.x,Twist_impact.linear.y,Twist_impact.linear.z);
    fprintf(fPtr,"%.3f,%.3f,%.3f,%.3f,",Pose_impact.orientation.x,Pose_impact.orientation.y,Pose_impact.orientation.z,Pose_impact.orientation.w);
    fprintf(fPtr,"%.3f,%.3f,%.3f,",Twist_impact.angular.x,Twist_impact.angular.y,Twist_impact.angular.z);
    fprintf(fPtr,"%.3f,%.3f,%.3f,",Eul_impact.x,Eul_impact.y,Eul_impact.z);


    // MISC RL LABELS
    fprintf(fPtr,"%s,%s,",formatBool(BodyContact_flag),formatBool(impact_flag));

    // MISC INTERNAL STATE ESTIMATES
    fprintf(fPtr,"--,--,--,"); // Tau_est,OF_x_est,OF_y_est
    fprintf(fPtr,"%u,%u,%u,%u,%u,",Pad_Connections,Pad1_Contact,Pad2_Contact,Pad3_Contact,Pad4_Contact);
    fprintf(fPtr,"--,%.3f,%.3f,%.3f,",impact_force_x,impact_force_y,impact_force_z);
    fprintf(fPtr,"--,--,--,--,");
    fprintf(fPtr,"--,--,--,--,");


    // SETPOINT VALUES
    fprintf(fPtr,"--,--,--,");
    fprintf(fPtr,"--,--,--,");
    fprintf(fPtr,"--,--,--,");


    // MISC VALUES
    fprintf(fPtr,"--,%s","Impact Data");
    fprintf(fPtr,"\n");
    fflush(fPtr);

}

void CF_DataConverter::append_CSV_blank()
{
    fprintf(fPtr,"\n");
    fflush(fPtr);
}

bool CF_DataConverter::DataLogging_Callback(crazyflie_msgs::loggingCMD::Request &req, crazyflie_msgs::loggingCMD::Response &res)
{
    // TURN ON/OFF LOGGING
    Logging_Flag = req.Logging_Flag;

    // CREATE CSV WHEN ACTIVATED
    if(req.createCSV == true)
    {   
        fPtr = fopen(req.filePath.c_str(), "w");
        create_CSV();
    }
    // CAP CSV W/ FLIP,IMPACT,MISC DATA
    else if(req.capLogging == true)
    {

        error_string = req.error_string;
        append_CSV_blank();
        append_CSV_misc();
        append_CSV_flip();
        append_CSV_impact();
        append_CSV_blank();
    }

    return 1;
}

