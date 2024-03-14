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
#include "sar_msgs/SAR_StateData.h"
#include "sar_msgs/SAR_TriggerData.h"
#include "sar_msgs/SAR_ImpactData.h"
#include "sar_msgs/SAR_MiscData.h"

#include "sar_msgs/CTRL_Data.h"
#include "sar_msgs/CTRL_Debug.h"
#include "sar_msgs/CTRL_Cmd_srv.h"
#include "sar_msgs/CTRL_Cmd.h"


#include "sar_msgs/RL_Data.h"
#include "sar_msgs/Sticky_Pad_Connect.h"

#include "sar_msgs/Activate_Sticky_Pads.h"
#include "sar_msgs/Logging_CMD.h"
#include "sar_msgs/GenericLogData.h"
#include "sar_msgs/Surface_Params.h"


#include "quatcompress.h"


class SAR_DataConverter {

    public:

        SAR_DataConverter(ros::NodeHandle* nh)
        {
            // DATA INPUT PIPELINE
            CTRL_Data_Sub = nh->subscribe("/CTRL/data", 1, &SAR_DataConverter::CtrlData_Callback, this, ros::TransportHints().tcpNoDelay());
            CTRL_Debug_Sub = nh->subscribe("/CTRL/debug", 1, &SAR_DataConverter::CtrlDebug_Callback, this, ros::TransportHints().tcpNoDelay());
            RL_Data_Sub = nh->subscribe("/RL/Data",5,&SAR_DataConverter::RL_Data_Callback,this,ros::TransportHints().tcpNoDelay());
            

            // GAZEBO PIPELINE
            GZ_SimSpeed_Client = nh->serviceClient<gazebo_msgs::SetPhysicsProperties>("/gazebo/set_physics_properties");
            Landing_Surface_Pose_Client = nh->serviceClient<sar_msgs::Surface_Params>("/ENV/Landing_Surface_Pose");

            Surface_ForceTorque_Sub = nh->subscribe("/ENV/Surface_ForceTorque_Sensor",5,&SAR_DataConverter::SurfaceFT_Sensor_Callback,this,ros::TransportHints().tcpNoDelay());
            Surface_Contact_Sub = nh->subscribe("/ENV/SurfaceContact",5,&SAR_DataConverter::Surface_Contact_Callback,this,ros::TransportHints().tcpNoDelay());
            SAR_Sticky_Pad_Connect_Sub = nh->subscribe("/SAR_Internal/Leg_Connections",5,&SAR_DataConverter::Pad_Connections_Callback,this,ros::TransportHints().tcpNoDelay());

            // CRAZYSWARM PIPELINE
            cf1_States_B_O_Sub = nh->subscribe("/cf1/States_B_O", 1, &SAR_DataConverter::cf1_States_B_O_Callback, this, ros::TransportHints().tcpNoDelay());
            cf1_States_B_P_Sub = nh->subscribe("/cf1/States_B_P", 1, &SAR_DataConverter::cf1_States_B_P_Callback, this, ros::TransportHints().tcpNoDelay());
            cf1_TrgState_Sub = nh->subscribe("/cf1/TrgState", 1, &SAR_DataConverter::cf1_TrgState_Callback, this, ros::TransportHints().tcpNoDelay());
            cf1_CTRL_Output_Sub = nh->subscribe("/cf1/CTRL_Output", 1, &SAR_DataConverter::cf1_CTRL_Output_Callback, this, ros::TransportHints().tcpNoDelay());
            cf1_SetPoints_Sub = nh->subscribe("/cf1/SetPoints", 1, &SAR_DataConverter::cf1_SetPoints_Callback, this, ros::TransportHints().tcpNoDelay());
            cf1_Flags_Sub = nh->subscribe("/cf1/Flags", 1, &SAR_DataConverter::cf1_Flags_Callback, this, ros::TransportHints().tcpNoDelay());
            cf1_Misc_Sub = nh->subscribe("/cf1/Misc", 1, &SAR_DataConverter::cf1_Misc_Callback, this, ros::TransportHints().tcpNoDelay());



            // INITIALIZE CTRL COMMAND PIPELINE
            CMD_Input_Service = nh->advertiseService("/SAR_DC/CMD_Input",&SAR_DataConverter::CMD_SAR_DC_Callback,this); // CTRL COMMAND
            CMD_Output_Service = nh->serviceClient<sar_msgs::CTRL_Cmd_srv>("/CTRL/Cmd_ctrl");          // Service client for sim controller
            CMD_Output_Topic = nh->advertise<sar_msgs::CTRL_Cmd>("/SAR_DC/CMD_Output_Topic",1);        // Msg publisher for Crazyswarm->CF->Controller


            // INITIALIZE STATE DATA PUBLISHERS
            StateData_Pub = nh->advertise<sar_msgs::SAR_StateData>("/SAR_DC/StateData",1);
            TriggerData_Pub =  nh->advertise<sar_msgs::SAR_TriggerData>("/SAR_DC/TriggerData",1);
            ImpactData_Pub = nh->advertise<sar_msgs::SAR_ImpactData>("/SAR_DC/ImpactData",1);  
            MiscData_Pub =  nh->advertise<sar_msgs::SAR_MiscData>("/SAR_DC/MiscData",1);

            // LOGGING 
            Logging_Service = nh->advertiseService("/SAR_DC/DataLogging", &SAR_DataConverter::DataLogging_Callback, this);



            // INITIALIZE SAR_DC THREADS
            SAR_DC_Thread = std::thread(&SAR_DataConverter::MainLoop, this);
            ConsoleOutput_Thread = std::thread(&SAR_DataConverter::ConsoleLoop, this);
            Logging_Thread = std::thread(&SAR_DataConverter::LoggingLoop, this);
            CrazyswarmPing_Thread = std::thread(&SAR_DataConverter::CrazyswarmPingLoop, this);


        }
        
        void MainInit();
        void MainLoop();
        void ConsoleLoop();
        void LoggingLoop();
        void CrazyswarmPingLoop();


        // =======================
        //     GAZEBO FUNCTIONS
        // =======================
        void activateStickyFeet();
        void checkSlowdown();
        void adjustSimSpeed(float speed_mult);
        void setLandingSurfacePose(float Pos_x, float Pos_y, float Pos_z, float Plane_Angle_deg);

        // =======================
        //     GAZEBO CALLBACKS
        // =======================
        void CtrlData_Callback(const sar_msgs::CTRL_Data &ctrl_msg);
        void CtrlDebug_Callback(const sar_msgs::CTRL_Debug &ctrl_msg);

        void SurfaceFT_Sensor_Callback(const geometry_msgs::WrenchStamped::ConstPtr &msg);
        void Surface_Contact_Callback(const gazebo_msgs::ContactsState &msg);
        void Pad_Connections_Callback(const sar_msgs::Sticky_Pad_Connect &msg);


        // =================================
        //     EXPERIMENT DATA CALLBACKS
        // =================================
        void decompressXY(uint32_t xy, float xy_arr[]);
        void cf1_States_B_O_Callback(const sar_msgs::GenericLogData::ConstPtr &log_msg);
        void cf1_States_B_P_Callback(const sar_msgs::GenericLogData::ConstPtr &log_msg);
        void cf1_CTRL_Output_Callback(const sar_msgs::GenericLogData::ConstPtr &log_msg);
        void cf1_SetPoints_Callback(const sar_msgs::GenericLogData::ConstPtr &log_msg);
        void cf1_TrgState_Callback(const sar_msgs::GenericLogData::ConstPtr &log_msg);
        void cf1_Flags_Callback(const sar_msgs::GenericLogData::ConstPtr &log_msg);
        void cf1_Misc_Callback(const sar_msgs::GenericLogData::ConstPtr &log_msg);


        // =============================
        //     CTRL COMMAND CALLBACKS
        // =============================
        inline bool CMD_SAR_DC_Callback(sar_msgs::CTRL_Cmd_srv::Request &req, sar_msgs::CTRL_Cmd_srv::Response &res);


        // =======================
        //    LOGGING FUNCTIONS
        // =======================
        bool DataLogging_Callback(sar_msgs::Logging_CMD::Request &req, sar_msgs::Logging_CMD::Response &res);
        void create_CSV();
        void append_CSV_states();
        void append_CSV_misc();
        void append_CSV_Trg();
        void append_CSV_impact();
        void append_CSV_blank();



        // =================================
        //     ORGANIZED DATA PUBLISHERS
        // =================================
        void RL_Data_Callback(const sar_msgs::RL_Data::ConstPtr &msg);
        void Publish_StateData();
        void Publish_TriggerData();
        void Publish_ImpactData();
        void Publish_MiscData();


        // =======================
        //     MISC. FUNCTIONS
        // =======================
        inline void quat2euler(float quat[], float eul[]);
        inline void euler2quat(float quat[],float eul[]);
        inline void loadInitParams();
        inline void updateParams();
        inline void resetStateData();
        inline void resetTriggerData();
        inline void resetImpactData();


        

    private:
    
        std::thread SAR_DC_Thread;
        std::thread ConsoleOutput_Thread;
        std::thread Logging_Thread;
        std::thread CrazyswarmPing_Thread;

        // =====================
        //     SYSTEM PARAMS
        // =====================
        std::string DATA_TYPE;  // Sim or Experiment Flag
        ros::Time Time_start;   // Initial time in UNIX notation
        int LOGGING_RATE = 20;  // Default Logging Rate
        bool SHOW_CONSOLE = true;


        // ==================
        //     SAR PARAMS
        // ==================
        std::string SAR_Type;
        std::string SAR_Config;
        std::string SAR_Type_str;
        std::string SAR_Config_str;
        
        std::string POLICY_TYPE;

        // DEFAULT INERTIA VALUES FOR BASE CRAZYFLIE
        float Mass = NAN; // [kg]
        float Ixx = NAN;  // [kg*m^2]
        float Iyy = NAN;  // [kg*m^2]
        float Izz = NAN;  // [kg*m^2]

        float P_kp_xy,P_kd_xy,P_ki_xy;
        float P_kp_z,P_kd_z,P_ki_z;
        float R_kp_xy,R_kd_xy,R_ki_xy;     
        float R_kp_z,R_kd_z,R_ki_z;

        float Gamma_eff = NAN;
        float L_eff = NAN;
        float K_Pitch = NAN;
        float K_Yaw = NAN;


        // ============================
        //     LANDING PLANE PARAMS
        // ============================
        std::string Plane_Config;
        geometry_msgs::Vector3 Plane_Pos; // Initial Plane Position
        float Plane_Angle_deg = NAN; // Initial Plane Angle [Deg]



        // ====================
        //     SIM VARIABLES
        // ====================
        int SLOWDOWN_TYPE = 0;
        bool LANDING_SLOWDOWN_FLAG;
        float SIM_SPEED = 0.5; 
        float SIM_SLOWDOWN_SPEED = 0.5;


        // =====================
        //     GAZEBO OBJECTS
        // =====================
        ros::Subscriber CTRL_Data_Sub;
        ros::Subscriber CTRL_Debug_Sub;

        ros::Subscriber Surface_ForceTorque_Sub;
        ros::Subscriber Surface_Contact_Sub;
        ros::Subscriber SAR_Sticky_Pad_Connect_Sub;

        ros::ServiceClient Landing_Surface_Pose_Client;
        ros::ServiceClient GZ_SimSpeed_Client;

        // ===========================
        //     CTRL COMMAND OBJECTS
        // ===========================
        ros::ServiceServer CMD_Input_Service;
        ros::ServiceServer CMD_Service_Dashboard;
        ros::ServiceClient CMD_Output_Service;
        ros::Publisher CMD_Output_Topic;

        // ============================
        //     DATA PUBLISH OBJECTS
        // ============================
        ros::Publisher StateData_Pub;
        ros::Publisher TriggerData_Pub;
        ros::Publisher ImpactData_Pub;
        ros::Publisher MiscData_Pub;

        sar_msgs::SAR_StateData StateData_msg;
        sar_msgs::SAR_TriggerData TriggerData_msg;
        sar_msgs::SAR_ImpactData ImpactData_msg;
        sar_msgs::SAR_MiscData MiscData_msg;

        // ===================================
        //     EXP COMPRESSED DATA OBJECTS
        // ===================================
        ros::Subscriber cf1_States_B_O_Sub;
        ros::Subscriber cf1_States_B_P_Sub;
        ros::Subscriber cf1_CTRL_Output_Sub;
        ros::Subscriber cf1_SetPoints_Sub;
        ros::Subscriber cf1_TrgState_Sub;
        ros::Subscriber cf1_Flags_Sub;
        ros::Subscriber cf1_Misc_Sub;


        


        

        // ===================
        //     FLIGHT DATA
        // ===================

        ros::Time Time;
        ros::Time Time_prev;

        geometry_msgs::Pose Pose_B_O;
        geometry_msgs::Twist Twist_B_O;
        geometry_msgs::Accel Accel_B_O;
        geometry_msgs::Vector3 Eul_B_O;
        double Vel_mag_B_O = NAN;
        double Vel_angle_B_O = NAN;
        float Accel_B_O_Mag = NAN;

        geometry_msgs::Pose Pose_P_B;
        geometry_msgs::Twist Twist_B_P;
        geometry_msgs::Vector3 Eul_P_B;

        double Vel_mag_B_P = NAN;
        double Vel_angle_B_P = NAN;
        double D_perp = NAN;
        double D_perp_CR = NAN;
        double D_perp_CR_min = INFINITY;   

        geometry_msgs::Vector3 Optical_Flow;
        geometry_msgs::Vector3 Optical_Flow_Cam;

        double Tau = NAN;
        double Tau_CR = NAN;
        double Theta_x = NAN;
        double Theta_y = NAN;

        double Tau_Cam = NAN;
        double Theta_x_Cam = NAN;
        double Theta_y_Cam = NAN;


        geometry_msgs::Vector3 x_d;
        geometry_msgs::Vector3 v_d;
        geometry_msgs::Vector3 a_d;


        boost::array<double,4> FM{0,0,0,0};
        boost::array<double,4> MotorThrusts{0,0,0,0};
        boost::array<uint16_t,4> Motor_CMD{0,0,0,0};

        boost::array<double,4> NN_Output{NAN,NAN,NAN,NAN};
        double a_Trg = NAN;
        double a_Rot = NAN;
        double Rot_Sum = 0.0;

        // ==========================
        //  STATES AT POLICY TRIGGER
        // ==========================

        bool Trg_Flag = false;
        bool OnceFlag_Trg = false;
        ros::Time Time_trg;

        geometry_msgs::Pose Pose_B_O_trg;
        geometry_msgs::Twist Twist_B_O_trg;
        geometry_msgs::Vector3 Eul_B_O_trg;
        double Vel_mag_B_O_trg = NAN;
        double Vel_angle_B_O_trg = NAN;

        geometry_msgs::Pose Pose_P_B_trg;
        geometry_msgs::Twist Twist_B_P_trg;
        geometry_msgs::Vector3 Eul_P_B_trg;

        double Vel_mag_B_P_trg = NAN;
        double Vel_angle_B_P_trg = NAN;
        double D_perp_trg = NAN;
        double D_perp_CR_trg = NAN;

        geometry_msgs::Vector3 Optical_Flow_trg;
        double Tau_trg = NAN;
        double Tau_CR_trg = NAN;
        double Theta_x_trg = NAN;
        double Theta_y_trg = NAN;

        boost::array<double,4> NN_Output_trg{NAN,NAN,NAN,NAN};
        double a_Trg_trg = NAN;
        double a_Rot_trg = NAN;

        // =======================
        //   ONBOARD IMPACT DATA
        // =======================
        bool Impact_Flag_OB = false;
        bool OnceFlag_Impact_OB = false;
        ros::Time Time_impact_OB;

        geometry_msgs::Pose Pose_B_O_impact_OB;
        geometry_msgs::Vector3 Eul_B_O_impact_OB;

        geometry_msgs::Twist Twist_B_P_impact_OB;
        geometry_msgs::Vector3 Eul_P_B_impact_OB;
        float Accel_B_O_Mag_impact_OB = NAN;


        // ==========================
        //    EXTERNAL IMPACT DATA
        // ==========================
        bool Impact_Flag_Ext = false;
        ros::Time Time_impact_Ext;

        geometry_msgs::Pose Pose_B_O_impact_Ext;
        geometry_msgs::Vector3 Eul_B_O_impact_Ext;

        geometry_msgs::Twist Twist_B_P_impact_Ext;
        geometry_msgs::Vector3 Eul_P_B_impact_Ext;
        float Rot_Sum_impact_Ext = NAN;

        bool BodyContact_Flag = false;
        bool ForelegContact_Flag = false;
        bool HindlegContact_Flag = false;
        bool OnceFlag_Impact = false;
        std::string BodyCollision_str = "SAR_Body::Body_Collision_";
        std::string LegCollision_str = "Leg_Collision_";


        // ==========================
        //    IMPACT FORCE DATA
        // ==========================
        bool Impact_Flag = false;
        geometry_msgs::Vector3 Force_impact;
        double Force_Impact_x = 0.0; // Max impact force in X-direction [N]
        double Force_Impact_y = 0.0; // Max impact force in Y-direction [N]
        double Force_Impact_z = 0.0; // Max impact force in Z-direction [N]
        double Impact_Magnitude = 0.0; // Current impact force magnitude

        // CIRCULAR BUFFERES TO LAG IMPACT STATE DATA (WE WANT STATE DATA THE INSTANT BEFORE IMPACT)
        boost::circular_buffer<geometry_msgs::Pose> Pose_B_O_impact_buff {4};
        boost::circular_buffer<geometry_msgs::Vector3> Eul_B_O_impact_buff {4};

        boost::circular_buffer<geometry_msgs::Twist> Twist_P_B_impact_buff {4};
        boost::circular_buffer<geometry_msgs::Vector3> Eul_P_B_impact_buff {4};

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

        bool Tumbled_Flag = false;
        bool TumbleDetect_Flag = false;
        bool MotorStop_Flag = false;
        bool AngAccel_Flag = false;
        bool Armed_Flag = false;
        bool CustomThrust_Flag = false;
        bool CustomMotorCMD_Flag = false;

        bool Pos_Ctrl_Flag = false;
        bool Vel_Ctrl_Flag = false;
        bool Policy_Armed_Flag = false;


        bool CamActive_Flag = false;

        // SIM
        bool Sticky_Flag = false;


        // ===================
        //     RL DATA
        // ===================
        ros::Subscriber RL_Data_Sub;
        uint8_t K_ep = 0;
        uint8_t K_run = 0;
        uint8_t n_rollouts = 8;

        boost::array<double,2> mu{0,0};
        boost::array<float,2> sigma{0,0};
        boost::array<float,2> policy{0,0};

        float reward = 0.0;
        boost::array<float,6> reward_vals{0,0,0,0,0,0};


        boost::array<float,3> vel_d{0,0,0};


        // =========================
        //     LOGGING VARIABLES
        // =========================
        ros::ServiceServer Logging_Service;
        FILE* fPtr; // File Pointer to logging file
        bool Logging_Flag = false;
        std::string error_string = "No_Data";

    
};

inline void SAR_DataConverter::loadInitParams()
{
    // SAR SETTINGS
    ros::param::get("/DATA_TYPE",DATA_TYPE);
    ros::param::get("/SAR_SETTINGS/SAR_Type",SAR_Type);
    ros::param::get("/SAR_SETTINGS/SAR_Config",SAR_Config);

    SAR_Type_str = "/SAR_Type/" + SAR_Type;
    SAR_Config_str = "/Config/" + SAR_Config;

    // UPDATE INTERTIAL PARAMETERS
    ros::param::get(SAR_Type_str + SAR_Config_str + "/Ref_Mass",Mass);
    ros::param::get(SAR_Type_str + SAR_Config_str + "/Ref_Ixx",Ixx);
    ros::param::get(SAR_Type_str + SAR_Config_str + "/Ref_Iyy",Iyy);
    ros::param::get(SAR_Type_str + SAR_Config_str + "/Ref_Izz",Izz);

    // UPDATE LEG PARAMETERS
    ros::param::get(SAR_Type_str + SAR_Config_str + "/Gamma_eff",Gamma_eff);
    ros::param::get(SAR_Type_str + SAR_Config_str + "/L_eff",L_eff);
    ros::param::get(SAR_Type_str + "/Leg_Params/K_Pitch",K_Pitch);
    ros::param::get(SAR_Type_str + "/Leg_Params/K_Yaw",K_Yaw);



    // PLANE SETTINGS
    ros::param::get("/PLANE_SETTINGS/Plane_Config",Plane_Config);

    
    // DATA SETTINGS
    if(DATA_TYPE.compare("SIM") == 0)
    {
        ros::param::set("/use_sim_time",true);
    }
    else
    {
        ros::param::set("/use_sim_time",false);
    }

    // UPDATE REMAINING PARAMS
    updateParams();
}

inline void SAR_DataConverter::updateParams()
{
    
    // UPDATE CTRL GAINS
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

    ros::param::get("/SAR_SETTINGS/Policy_Type",POLICY_TYPE);


    // DEBUG SETTINGS
    ros::param::get("/SIM_SETTINGS/Sim_Speed",SIM_SPEED);
    ros::param::get("/SIM_SETTINGS/Sim_Slowdown_Speed",SIM_SLOWDOWN_SPEED);
    ros::param::get("/SIM_SETTINGS/Landing_Slowdown_Flag",LANDING_SLOWDOWN_FLAG);

    ros::param::get("/SAR_DC_SETTINGS/Logging_Rate",LOGGING_RATE);
    ros::param::get("/SAR_DC_SETTINGS/Console_Output",SHOW_CONSOLE);

}

inline void SAR_DataConverter::resetStateData()
{
    D_perp_CR_min = INFINITY;
}

inline void SAR_DataConverter::resetTriggerData()
{
    Trg_Flag = false;
    OnceFlag_Trg = false;
    Time_trg.sec = 0.0;
    Time_trg.nsec = 0.0;

    // STATES WRT ORIGIN
    Pose_B_O_trg = geometry_msgs::Pose();
    Twist_B_O_trg = geometry_msgs::Twist();
    Eul_B_O_trg = geometry_msgs::Vector3();

    // STATES WRT PLANE
    Pose_P_B_trg = geometry_msgs::Pose();
    Twist_B_P_trg = geometry_msgs::Twist();
    Eul_P_B_trg = geometry_msgs::Vector3();
    Vel_mag_B_P_trg = NAN;
    Vel_angle_B_P_trg = NAN;
    D_perp_trg = NAN;
    D_perp_CR_trg = NAN;

    // OPTICAL FLOW
    Optical_Flow_trg = geometry_msgs::Vector3();
    Tau_CR_trg = NAN;

    // POLICY ACTIONS
    a_Trg_trg = NAN;
    a_Rot_trg = NAN;
}

inline void SAR_DataConverter::resetImpactData()
{
    Impact_Flag = false;

    // ONBOARD IMPACT DATA
    Impact_Flag_OB = false;
    OnceFlag_Impact_OB = false;
    Time_impact_OB.sec = 0.0;
    Time_impact_OB.nsec = 0.0;

    Pose_B_O_impact_OB = geometry_msgs::Pose();
    Eul_B_O_impact_OB = geometry_msgs::Vector3();

    Twist_B_P_impact_OB = geometry_msgs::Twist();
    Eul_P_B_impact_OB = geometry_msgs::Vector3();
    Accel_B_O_Mag_impact_OB = NAN;

    // EXTERNAL IMPACT DATA
    Impact_Flag_Ext = false;
    Time_impact_Ext.sec = 0.0;
    Time_impact_Ext.nsec = 0.0;

    BodyContact_Flag = false;
    ForelegContact_Flag = false;
    HindlegContact_Flag = false;

    Pose_B_O_impact_Ext = geometry_msgs::Pose();
    Eul_B_O_impact_Ext = geometry_msgs::Vector3();

    Twist_B_P_impact_Ext = geometry_msgs::Twist();
    Eul_P_B_impact_Ext = geometry_msgs::Vector3();
    Rot_Sum = 0.0;
    Rot_Sum_impact_Ext = NAN;

    // IMPACT FORCE DATA
    Force_impact = geometry_msgs::Vector3();
    Impact_Magnitude = 0.0;

    // STICKY PAD CONTACTS
    Pad_Connections = 0;
    Pad1_Contact = 0;
    Pad2_Contact = 0;
    Pad3_Contact = 0;
    Pad4_Contact = 0;

}

inline bool SAR_DataConverter::CMD_SAR_DC_Callback(sar_msgs::CTRL_Cmd_srv::Request &req, sar_msgs::CTRL_Cmd_srv::Response &res)
{
    switch (req.cmd_type)
    {
        case 0:

            resetStateData();
            resetTriggerData();
            resetImpactData();

            if (DATA_TYPE.compare("SIM") == 0)
            {
                // RESET SIM SPEED
                SAR_DataConverter::adjustSimSpeed(SIM_SPEED);
                SLOWDOWN_TYPE = 0;
            }
            break;

        case 9: // UPDATE PLANE POSITION
            SAR_DataConverter::setLandingSurfacePose(req.cmd_vals.x,req.cmd_vals.y,req.cmd_vals.z,req.cmd_flag);            
            break;

        case 21:  // UPDATE PARAMS IN SAR_DC 
            SAR_DataConverter::loadInitParams();
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

        

        default:
            break;
    }


    // SIMULATION: SEND COMMAND VALUES TO SIM CONTROLLER (SEND AS SERVICE REQUEST)
    if (DATA_TYPE.compare("SIM") == 0)
    {
        sar_msgs::CTRL_Cmd_srv srv;
        srv.request = req;
        CMD_Output_Service.call(srv);
        return srv.response.srv_Success; // Return if service request successful (true/false)
    }
    else
    {
        // EXPERIMENT: SEND COMMAND VALUES TO PHYSICAL CONTROLLER (BROADCAST CMD VALUES AS ROS MESSAGE)
        sar_msgs::CTRL_Cmd cmd_msg;
        cmd_msg.cmd_type = req.cmd_type;
        cmd_msg.cmd_vals = req.cmd_vals;
        cmd_msg.cmd_flag = req.cmd_flag;
        cmd_msg.cmd_rx = req.cmd_rx;

        for (int i = 0; i < 3; i++)
        {
            CMD_Output_Topic.publish(cmd_msg);
        }

        return true;
    }
    
    
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

