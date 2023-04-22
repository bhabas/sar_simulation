#pragma once


// STANDARD INCLUDES
#include <stdio.h>
#include <iostream>
#include <boost/circular_buffer.hpp>
#include <math.h>       /* sqrt */
#include <thread>
#include <locale.h>

#include <ncurses.h>
#include <unistd.h>
#include <ctime>

// ROS INCLUDES
#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Point.h>

#include <gazebo_msgs/ContactsState.h>
#include <gazebo_msgs/SetPhysicsProperties.h>
#include <gazebo_msgs/SetModelState.h>

// CUSTOM INCLUDES
#include "crazyflie_msgs/CF_StateData.h"
#include "crazyflie_msgs/CF_FlipData.h"
#include "crazyflie_msgs/CF_ImpactData.h"
#include "crazyflie_msgs/CF_MiscData.h"

#include "crazyflie_msgs/CtrlData.h"
#include "crazyflie_msgs/CtrlDebug.h"
#include "crazyflie_msgs/GTC_Cmd_srv.h"
#include "crazyflie_msgs/GTC_Cmd.h"


#include "crazyflie_msgs/RLData.h"
#include "crazyflie_msgs/PadConnect.h"

#include "crazyflie_msgs/activateSticky.h"
#include "crazyflie_msgs/loggingCMD.h"
#include "crazyflie_msgs/GenericLogData.h"

#include "quatcompress.h"


class SAR_DataConverter {

    public:

        SAR_DataConverter(ros::NodeHandle* nh)
        {
            // DATA INPUT PIPELINE
            CTRL_Data_Sub = nh->subscribe("/CTRL/data", 1, &SAR_DataConverter::CtrlData_Callback, this, ros::TransportHints().tcpNoDelay());
            CTRL_Debug_Sub = nh->subscribe("/CTRL/debug", 1, &SAR_DataConverter::CtrlDebug_Callback, this, ros::TransportHints().tcpNoDelay());
            RL_Data_Sub = nh->subscribe("/RL/data",5,&SAR_DataConverter::RL_Data_Callback,this,ros::TransportHints().tcpNoDelay());
            

            // GAZEBO PIPELINE
            GZ_SimSpeed_Client = nh->serviceClient<gazebo_msgs::SetPhysicsProperties>("/gazebo/set_physics_properties");
            Landing_Surface_Pose_Client = nh->serviceClient<gazebo_msgs::SetModelState>("/Landing_Surface_Pose");

            // CRAZYSWARM PIPELINE
            log1_Sub = nh->subscribe("/cf1/log1", 1, &SAR_DataConverter::log1_Callback, this, ros::TransportHints().tcpNoDelay());
            log2_Sub = nh->subscribe("/cf1/log2", 1, &SAR_DataConverter::log2_Callback, this, ros::TransportHints().tcpNoDelay());
            log3_Sub = nh->subscribe("/cf1/log3", 1, &SAR_DataConverter::log3_Callback, this, ros::TransportHints().tcpNoDelay());
            log4_Sub = nh->subscribe("/cf1/log4", 1, &SAR_DataConverter::log4_Callback, this, ros::TransportHints().tcpNoDelay());
            log5_Sub = nh->subscribe("/cf1/log5", 1, &SAR_DataConverter::log5_Callback, this, ros::TransportHints().tcpNoDelay());
            log6_Sub = nh->subscribe("/cf1/log6", 1, &SAR_DataConverter::log6_Callback, this, ros::TransportHints().tcpNoDelay());


            // INITIALIZE GTC COMMAND PIPELINE
            CMD_Input_Service = nh->advertiseService("/SAR_DC/CMD_Input",&SAR_DataConverter::CMD_SAR_DC_Callback,this); // GTC COMMAND
            CMD_Output_Topic = nh->advertise<crazyflie_msgs::GTC_Cmd>("/SAR_DC/CMD_Output_Topic",1);        // Msg publisher for Crazyswarm->CF->Controller
            CMD_Output_Service = nh->serviceClient<crazyflie_msgs::GTC_Cmd_srv>("/CTRL/Cmd_ctrl");          // Service client for sim controller


            // INITIALIZE STATE DATA PUBLISHERS
            StateData_Pub = nh->advertise<crazyflie_msgs::CF_StateData>("/SAR_DC/StateData",1);
            MiscData_Pub =  nh->advertise<crazyflie_msgs::CF_MiscData>("/SAR_DC/MiscData",1);
            FlipData_Pub =  nh->advertise<crazyflie_msgs::CF_FlipData>("/SAR_DC/FlipData",1);
            ImpactData_Pub = nh->advertise<crazyflie_msgs::CF_ImpactData>("/SAR_DC/ImpactData",1);  

            // LOGGING 
            Logging_Service = nh->advertiseService("/SAR_DC/DataLogging", &SAR_DataConverter::DataLogging_Callback, this);



            // INITIALIZE SAR_DC THREADS
            SAR_DC_Thread = std::thread(&SAR_DataConverter::MainLoop, this);
            ConsoleOutput_Thread = std::thread(&SAR_DataConverter::ConsoleLoop, this);
            Logging_Thread = std::thread(&SAR_DataConverter::LoggingLoop, this);


        }

        void MainInit();
        void MainLoop();
        void ConsoleLoop();
        void LoggingLoop();


        // =======================
        //     GAZEBO FUNCTIONS
        // =======================
        void activateStickyFeet();
        void checkSlowdown();
        void adjustSimSpeed(float speed_mult);
        void Update_Landing_Surface_Pose(float Pos_x, float Pos_y, float Pos_z, float Plane_Angle);

        // =======================
        //     GAZEBO CALLBACKS
        // =======================
        void CtrlData_Callback(const crazyflie_msgs::CtrlData &ctrl_msg);
        void CtrlDebug_Callback(const crazyflie_msgs::CtrlDebug &ctrl_msg);

        void SurfaceFT_Sensor_Callback(const geometry_msgs::WrenchStamped::ConstPtr &msg);
        void Surface_Contact_Callback(const gazebo_msgs::ContactsState &msg);
        void Pad_Connections_Callback(const crazyflie_msgs::PadConnect &msg);


        // =================================
        //     EXPERIMENT DATA CALLBACKS
        // =================================
        void decompressXY(uint32_t xy, float xy_arr[]);
        void log1_Callback(const crazyflie_msgs::GenericLogData::ConstPtr &log1_msg);
        void log2_Callback(const crazyflie_msgs::GenericLogData::ConstPtr &log2_msg);
        void log3_Callback(const crazyflie_msgs::GenericLogData::ConstPtr &log3_msg);
        void log4_Callback(const crazyflie_msgs::GenericLogData::ConstPtr &log4_msg);
        void log5_Callback(const crazyflie_msgs::GenericLogData::ConstPtr &log5_msg);
        void log6_Callback(const crazyflie_msgs::GenericLogData::ConstPtr &log6_msg);

        // =============================
        //     GTC COMMAND CALLBACKS
        // =============================
        inline bool CMD_SAR_DC_Callback(crazyflie_msgs::GTC_Cmd_srv::Request &req, crazyflie_msgs::GTC_Cmd_srv::Response &res);
        inline bool Send_Cmd2Ctrl(crazyflie_msgs::GTC_Cmd_srv::Request &req);


        // =======================
        //    LOGGING FUNCTIONS
        // =======================
        bool DataLogging_Callback(crazyflie_msgs::loggingCMD::Request &req, crazyflie_msgs::loggingCMD::Response &res);
        void create_CSV();
        void append_CSV_states();
        void append_CSV_misc();
        void append_CSV_flip();
        void append_CSV_impact();
        void append_CSV_blank();



        // =================================
        //     ORGANIZED DATA PUBLISHERS
        // =================================
        void RL_Data_Callback(const crazyflie_msgs::RLData::ConstPtr &msg);
        void Publish_StateData();
        void Publish_FlipData();
        void Publish_ImpactData();
        void Publish_MiscData();


        // =======================
        //     MISC. FUNCTIONS
        // =======================
        inline void quat2euler(float quat[], float eul[]);
        inline void euler2quat(float quat[],float eul[]);
        inline void LoadParams();


        

    private:
    
        std::thread SAR_DC_Thread;
        std::thread ConsoleOutput_Thread;
        std::thread Logging_Thread;

        // =====================
        //     SYSTEM PARAMS
        // =====================
        std::string DATA_TYPE;  // Sim or Experiment Flag
        ros::Time Time_start;   // Initial time in UNIX notation
        int LOGGING_RATE = 10;  // Default Logging Rate
        bool isInit = false;    // Load these params only on first start of SAR_DC


        // ==================
        //     SAR PARAMS
        // ==================
        std::string SAR_Type;
        std::string SAR_Config;
        std::string GZ_Model_Name;
        std::string POLICY_TYPE;

        // DEFAULT INERTIA VALUES FOR BASE CRAZYFLIE
        float Mass = 34.4e3; // [kg]
        float Ixx = 15.83e-6f;  // [kg*m^2]
        float Iyy = 17.00e-6f;  // [kg*m^2]
        float Izz = 31.19e-6f;  // [kg*m^2]

        float P_kp_xy,P_kd_xy,P_ki_xy;
        float P_kp_z,P_kd_z,P_ki_z;
        float R_kp_xy,R_kd_xy,R_ki_xy;     
        float R_kp_z,R_kd_z,R_ki_z;


        // ============================
        //     LANDING PLANE PARAMS
        // ============================
        std::string Plane_Model;
        geometry_msgs::Vector3 Plane_Pos; // Initial Plane Position
        float Plane_Angle = 180.0; // Initial Plane Angle [Deg]



        // ====================
        //     SIM VARIABLES
        // ====================
        int SLOWDOWN_TYPE = 0;
        bool LANDING_SLOWDOWN_FLAG;
        float SIM_SPEED; 
        float SIM_SLOWDOWN_SPEED;


        // =====================
        //     GAZEBO OBJECTS
        // =====================
        ros::Subscriber CTRL_Data_Sub;
        ros::Subscriber CTRL_Debug_Sub;

        ros::Subscriber Surface_FT_Sub;
        ros::Subscriber Surface_Contact_Sub;
        ros::Subscriber PadConnect_Sub;

        ros::ServiceClient Landing_Surface_Pose_Client;
        ros::ServiceClient GZ_SimSpeed_Client;

        // ===========================
        //     GTC COMMAND OBJECTS
        // ===========================
        ros::ServiceServer CMD_Input_Service;
        ros::ServiceServer CMD_Service_Dashboard;
        ros::ServiceClient CMD_Output_Service;
        ros::Publisher CMD_Output_Topic;

        // ============================
        //     DATA PUBLISH OBJECTS
        // ============================
        ros::Publisher StateData_Pub;
        ros::Publisher FlipData_Pub;
        ros::Publisher ImpactData_Pub;
        ros::Publisher MiscData_Pub;

        crazyflie_msgs::CF_StateData StateData_msg;
        crazyflie_msgs::CF_FlipData FlipData_msg;
        crazyflie_msgs::CF_ImpactData ImpactData_msg;
        crazyflie_msgs::CF_MiscData MiscData_msg;

        // ===================================
        //     EXP COMPRESSED DATA OBJECTS
        // ===================================
        ros::Subscriber log1_Sub;
        ros::Subscriber log2_Sub;
        ros::Subscriber log3_Sub;
        ros::Subscriber log4_Sub;
        ros::Subscriber log5_Sub;
        ros::Subscriber log6_Sub;

        


        

        // ===================
        //     FLIGHT DATA
        // ===================

        ros::Time Time;

        geometry_msgs::Pose Pose;
        geometry_msgs::Twist Twist;
        geometry_msgs::Vector3 Eul;

        float Vel_mag = 0.0;
        float Phi = 0.0;
        float Alpha = 0.0;

        double Tau = 0.0;
        double Theta_x = 0.0;
        double Theta_y = 0.0;
        double D_perp = 0.0;

        double Tau_est = 0.0;
        double Theta_x_est = 0.0;
        double Theta_y_est = 0.0;

        boost::array<double,4> FM{0,0,0,0};
        boost::array<double,4> MotorThrusts{0,0,0,0};
        boost::array<uint16_t,4> MS_PWM{0,0,0,0};

        double Tau_thr = 0.0;
        double G1 = 0.0;

        double Policy_Flip = 0.0;
        double Policy_Action = 0.0;

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
        double Theta_x_tr = 0.0;
        double Theta_y_tr = 0.0;
        double D_perp_tr = 0.0;

        boost::array<double,4> FM_tr{0,0,0,0};

        double Policy_Flip_tr = 0.0;
        double Policy_Action_tr = 0.0;


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
        ros::Subscriber RL_Data_Sub;
        uint8_t k_ep = 0;
        uint8_t k_run = 0;
        uint8_t n_rollouts = 8;

        boost::array<double,2> mu{0,0};
        boost::array<float,2> sigma{0,0};
        boost::array<float,3> policy{0,0,0};

        float reward = 0.0;
        boost::array<float,5> reward_vals{0,0,0,0,0};


        boost::array<float,3> vel_d{0,0,0};


        // =========================
        //     LOGGING VARIABLES
        // =========================
        ros::ServiceServer Logging_Service;
        FILE* fPtr; // File Pointer to logging file
        bool Logging_Flag = false;
        std::string error_string = "No_Data";

    
};


inline void SAR_DataConverter::LoadParams()
{
    // QUAD SETTINGS
    ros::param::get("/QUAD_SETTINGS/SAR_Type",SAR_Type);
    ros::param::get("/QUAD_SETTINGS/SAR_Config",SAR_Config);
    ros::param::get("/QUAD_SETTINGS/Policy_Type",POLICY_TYPE);

    GZ_Model_Name = "crazyflie_" + SAR_Config;
    std::string SAR_Type_str = "/SAR_Type/" + SAR_Type;
    std::string SAR_Config_str = "/Config/" + SAR_Config;

    // PLANE SETTINGS
    ros::param::get("/PLANE_SETTINGS/Plane_Model",Plane_Model);
    if (isInit == false)
    {
        ros::param::get("/PLANE_SETTINGS/Plane_Angle",Plane_Angle);
        ros::param::get("/PLANE_SETTINGS/Pos_X",Plane_Pos.x);
        ros::param::get("/PLANE_SETTINGS/Pos_Y",Plane_Pos.y);
        ros::param::get("/PLANE_SETTINGS/Pos_Z",Plane_Pos.z);
    }
    
    


    // COLLECT MODEL PARAMETERS
    ros::param::get(SAR_Type_str + SAR_Config_str + "/Mass",Mass);
    ros::param::get(SAR_Type_str + SAR_Config_str + "/Ixx",Ixx);
    ros::param::get(SAR_Type_str + SAR_Config_str + "/Iyy",Iyy);
    ros::param::get(SAR_Type_str + SAR_Config_str + "/Izz",Izz);

    // DEBUG SETTINGS
    ros::param::get("/DATA_TYPE",DATA_TYPE);
    ros::param::get("/SIM_SETTINGS/Sim_Speed",SIM_SPEED);
    ros::param::get("/SIM_SETTINGS/Sim_Slowdown_Speed",SIM_SLOWDOWN_SPEED);
    ros::param::get("/SIM_SETTINGS/Landing_Slowdown_Flag",LANDING_SLOWDOWN_FLAG);

    ros::param::get("/CF_DC_SETTINGS/Logging_Rate",LOGGING_RATE);

    // COLLECT CTRL GAINS
    ros::param::get(SAR_Type_str + "/CtrlGains/P_kp_xy",P_kp_xy);
    ros::param::get(SAR_Type_str + "/CtrlGains/P_kd_xy",P_kd_xy);
    ros::param::get(SAR_Type_str + "/CtrlGains/P_ki_xy",P_ki_xy);

    ros::param::get(SAR_Type_str + "/CtrlGains/P_kp_z",P_kp_z);
    ros::param::get(SAR_Type_str + "/CtrlGains/P_kd_z",P_kd_z);
    ros::param::get(SAR_Type_str + "/CtrlGains/P_ki_z",P_ki_z);

    ros::param::get(SAR_Type_str + "/CtrlGains/R_kp_xy",R_kp_xy);
    ros::param::get(SAR_Type_str + "/CtrlGains/R_kd_xy",R_kd_xy);
    ros::param::get(SAR_Type_str + "/CtrlGains/R_ki_xy",R_ki_xy);
    
    ros::param::get(SAR_Type_str + "/CtrlGains/R_kp_z",R_kp_z);
    ros::param::get(SAR_Type_str + "/CtrlGains/R_kd_z",R_kd_z);
    ros::param::get(SAR_Type_str + "/CtrlGains/R_ki_z",R_ki_z);

    if(DATA_TYPE.compare("SIM") == 0)
    {
        ros::param::set("/use_sim_time",true);
    }
    else
    {
        ros::param::set("/use_sim_time",false);
    }

}

inline bool SAR_DataConverter::CMD_SAR_DC_Callback(crazyflie_msgs::GTC_Cmd_srv::Request &req, crazyflie_msgs::GTC_Cmd_srv::Response &res)
{
    // PASS COMMAND VALUES TO CONTROLLER AND PASS LOCAL ACTIONS
    SAR_DataConverter::Send_Cmd2Ctrl(req);
    res.srv_Success = true;
    return res.srv_Success;
}

inline bool SAR_DataConverter::Send_Cmd2Ctrl(crazyflie_msgs::GTC_Cmd_srv::Request &req)
{
    switch (req.cmd_type)
    {
        case 0:
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
                SAR_DataConverter::adjustSimSpeed(SIM_SPEED);
                SLOWDOWN_TYPE = 0;
            }
            break;

        case 21:  // UPDATE PARAMS IN CF_DC 
            SAR_DataConverter::LoadParams();
            break;
        
        case 92: // ACTIVATE STICKY FEET

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
                
                SAR_DataConverter::activateStickyFeet();
            }
            break;

        case 93: // UPDATE PLANE POSITION
            SAR_DataConverter::Update_Landing_Surface_Pose(req.cmd_vals.x,req.cmd_vals.y,req.cmd_vals.z,req.cmd_flag);

            Plane_Pos.x = req.cmd_vals.x;
            Plane_Pos.y = req.cmd_vals.y;
            Plane_Pos.z = req.cmd_vals.z;
            Plane_Angle = req.cmd_flag;
            
            break;

        default:
            break;
    }


    // SIMULATION:
    // SEND COMMAND VALUES TO SIM CONTROLLER
    crazyflie_msgs::GTC_Cmd_srv srv;
    srv.request = req;
    CMD_Output_Service.call(srv);


    // EXPERIMENT: 
    // SEND COMMAND VALUES TO PHYSICAL CONTROLLER
    // BROADCAST CMD VALUES AS ROS MESSAGE
    crazyflie_msgs::GTC_Cmd cmd_msg;
    cmd_msg.cmd_type = req.cmd_type;
    cmd_msg.cmd_vals = req.cmd_vals;
    cmd_msg.cmd_flag = req.cmd_flag;
    cmd_msg.cmd_rx = req.cmd_rx;
    CMD_Output_Topic.publish(cmd_msg);

    return srv.response.srv_Success; // Return if service request successful (true/false)
}


// CONVERT QUATERNION TO EULER ANGLES (YZX NOTATION)
inline void SAR_DataConverter::quat2euler(float quat[], float eul[]){

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


// CONVERTS A SET OF EULER ANGLES TO QUATERNION IN (ZYX NOTATION)
inline void SAR_DataConverter::euler2quat(float quat[],float eul[]) {

	// Abbreviations for the various angular functions

    double cx = cos(eul[0] * 0.5);
    double cy = cos(eul[1] * 0.5);
    double cz = cos(eul[2] * 0.5);

    double sx = sin(eul[0] * 0.5);
    double sy = sin(eul[1] * 0.5);
    double sz = sin(eul[2] * 0.5);

    quat[0] = sx * cy * cz - cx * sy * sz;
    quat[1] = cx * sy * cz + sx * cy * sz;
    quat[2] = cz * cz * sz - sx * sy * cz;
    quat[3] = cx * cy * cz + sx * sy * sz;

}

