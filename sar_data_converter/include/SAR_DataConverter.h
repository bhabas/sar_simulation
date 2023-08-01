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
#include "sar_msgs/PadConnect.h"

#include "sar_msgs/activateSticky.h"
#include "sar_msgs/loggingCMD.h"
#include "sar_msgs/GenericLogData.h"

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
            Landing_Surface_Pose_Client = nh->serviceClient<gazebo_msgs::SetModelState>("/Landing_Surface_Pose_Plugin");

            Surface_ForceTorque_Sub = nh->subscribe("/ENV/Surface_ForceTorque_Sensor",5,&SAR_DataConverter::SurfaceFT_Sensor_Callback,this,ros::TransportHints().tcpNoDelay());
            Surface_Contact_Sub = nh->subscribe("/ENV/BodyContact",5,&SAR_DataConverter::Surface_Contact_Callback,this,ros::TransportHints().tcpNoDelay());
            SAR_PadConnect_Sub = nh->subscribe("/ENV/Pad_Connections",5,&SAR_DataConverter::Pad_Connections_Callback,this,ros::TransportHints().tcpNoDelay());

            // CRAZYSWARM PIPELINE
            cf1_FullState_Sub = nh->subscribe("/cf1/FullState", 1, &SAR_DataConverter::cf1_FullState_Callback, this, ros::TransportHints().tcpNoDelay());
            cf1_PolicyState_Sub = nh->subscribe("/cf1/PolicyState", 1, &SAR_DataConverter::cf1_PolicyState_Callback, this, ros::TransportHints().tcpNoDelay());
            cf1_CTRL_Output_Sub = nh->subscribe("/cf1/CTRL_Output", 1, &SAR_DataConverter::cf1_CTRL_Output_Callback, this, ros::TransportHints().tcpNoDelay());
            cf1_SetPoints_Sub = nh->subscribe("/cf1/SetPoints", 1, &SAR_DataConverter::cf1_SetPoints_Callback, this, ros::TransportHints().tcpNoDelay());
            cf1_TrgState_Sub = nh->subscribe("/cf1/TrgState", 1, &SAR_DataConverter::cf1_TrgState_Callback, this, ros::TransportHints().tcpNoDelay());
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
        void CtrlData_Callback(const sar_msgs::CTRL_Data &ctrl_msg);
        void CtrlDebug_Callback(const sar_msgs::CTRL_Debug &ctrl_msg);

        void SurfaceFT_Sensor_Callback(const geometry_msgs::WrenchStamped::ConstPtr &msg);
        void Surface_Contact_Callback(const gazebo_msgs::ContactsState &msg);
        void Pad_Connections_Callback(const sar_msgs::PadConnect &msg);


        // =================================
        //     EXPERIMENT DATA CALLBACKS
        // =================================
        void decompressXY(uint32_t xy, float xy_arr[]);
        void cf1_FullState_Callback(const sar_msgs::GenericLogData::ConstPtr &log_msg);
        void cf1_PolicyState_Callback(const sar_msgs::GenericLogData::ConstPtr &log_msg);
        void cf1_CTRL_Output_Callback(const sar_msgs::GenericLogData::ConstPtr &log_msg);
        void cf1_SetPoints_Callback(const sar_msgs::GenericLogData::ConstPtr &log_msg);
        void cf1_TrgState_Callback(const sar_msgs::GenericLogData::ConstPtr &log_msg);
        void cf1_Flags_Callback(const sar_msgs::GenericLogData::ConstPtr &log_msg);
        void cf1_Misc_Callback(const sar_msgs::GenericLogData::ConstPtr &log_msg);


        // =============================
        //     CTRL COMMAND CALLBACKS
        // =============================
        inline bool CMD_SAR_DC_Callback(sar_msgs::CTRL_Cmd_srv::Request &req, sar_msgs::CTRL_Cmd_srv::Response &res);
        inline bool Send_Cmd2Ctrl(sar_msgs::CTRL_Cmd_srv::Request &req);


        // =======================
        //    LOGGING FUNCTIONS
        // =======================
        bool DataLogging_Callback(sar_msgs::loggingCMD::Request &req, sar_msgs::loggingCMD::Response &res);
        void create_CSV();
        void append_CSV_states();
        void append_CSV_misc();
        void append_CSV_flip();
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
        bool SHOW_CONSOLE = true;
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

        ros::Subscriber Surface_ForceTorque_Sub;
        ros::Subscriber Surface_Contact_Sub;
        ros::Subscriber SAR_PadConnect_Sub;

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
        ros::Subscriber cf1_FullState_Sub;
        ros::Subscriber cf1_PolicyState_Sub;
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

        geometry_msgs::Pose Pose;
        geometry_msgs::Twist Twist;
        geometry_msgs::Vector3 Eul;

        float Vel_mag = 0.0;
        float Phi = 0.0;
        float Alpha = 0.0;

        double D_perp = 0.0;
        double V_perp = 0.0;
        double V_tx = 0.0;
        double V_ty = 0.0;


        double Tau = 0.0;
        double Theta_x = 0.0;
        double Theta_y = 0.0;

        double Tau_est = 0.0;
        double Theta_x_est = 0.0;
        double Theta_y_est = 0.0;

        boost::array<double,4> FM{0,0,0,0};
        boost::array<double,4> MotorThrusts{0,0,0,0};
        boost::array<uint16_t,4> MS_PWM{0,0,0,0};

        double Policy_Trg_Action = 0.0;
        double Policy_Flip_Action = 0.0;

        geometry_msgs::Vector3 x_d;
        geometry_msgs::Vector3 v_d;
        geometry_msgs::Vector3 a_d;

        double Rot_Sum = 0.0;

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

        double Policy_Trg_Action_tr = 0.0;
        double Policy_Flip_Action_tr = 0.0;


        // ===================
        //     IMPACT DATA
        // ===================

        bool impact_flag = false;
        bool BodyContact_flag = false;
        bool OnceFlag_impact = false;
        std::string BodyCollision_str;  // String of Body Name


        ros::Time Time_impact;
        geometry_msgs::Vector3 Force_impact;
        geometry_msgs::Pose Pose_impact;
        geometry_msgs::Twist Twist_impact;
        geometry_msgs::Vector3 Eul_impact;


        double impact_force_x = 0.0; // Max impact force in X-direction [N]
        double impact_force_y = 0.0; // Max impact force in Y-direction [N]
        double impact_force_z = 0.0; // Max impact force in Z-direction [N]
        double impact_magnitude = 0.0; // Current impact force magnitude

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

        uint8_t Pad_Connect_Sum = 0;

        // ====================
        //     DEBUG VALUES
        // ====================

        bool Motorstop_Flag = false;
        bool Pos_Ctrl_Flag = false;
        bool Vel_Ctrl_Flag = false;
        bool Tumble_Detection = false;
        bool Tumbled_Flag = false;

        bool AttCtrl_Flag = false;
        bool Moment_Flag = false;
        bool CustomThrust_Flag = false;
        bool CustomPWM_Flag = false;

        bool Traj_Active_Flag = false;
        bool Policy_Armed_Flag = false;
        bool isCamActive = false;

        // SIM
        bool Sticky_Flag = false;

        // EXPERIMENT
        bool SafeModeEnable = false;


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
    // SAR SETTINGS
    ros::param::get("/SAR_SETTINGS/SAR_Type",SAR_Type);
    ros::param::get("/SAR_SETTINGS/SAR_Config",SAR_Config);

    GZ_Model_Name = SAR_Type + "_" + SAR_Config;
    std::string SAR_Type_str = "/SAR_Type/" + SAR_Type;
    std::string SAR_Config_str = "/Config/" + SAR_Config;

    // UPDATE INTERTIAL PARAMETERS
    ros::param::get(SAR_Type_str + SAR_Config_str + "/Mass",Mass);
    ros::param::get(SAR_Type_str + SAR_Config_str + "/Ixx",Ixx);
    ros::param::get(SAR_Type_str + SAR_Config_str + "/Iyy",Iyy);
    ros::param::get(SAR_Type_str + SAR_Config_str + "/Izz",Izz);


    // PLANE SETTINGS
    ros::param::get("/PLANE_SETTINGS/Plane_Model",Plane_Model);
    if (isInit == false)
    {
        ros::param::get("/PLANE_SETTINGS/Plane_Angle",Plane_Angle);
        ros::param::get("/PLANE_SETTINGS/Pos_X",Plane_Pos.x);
        ros::param::get("/PLANE_SETTINGS/Pos_Y",Plane_Pos.y);
        ros::param::get("/PLANE_SETTINGS/Pos_Z",Plane_Pos.z);
    }

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
    ros::param::get("/DATA_TYPE",DATA_TYPE);
    ros::param::get("/SIM_SETTINGS/Sim_Speed",SIM_SPEED);
    ros::param::get("/SIM_SETTINGS/Sim_Slowdown_Speed",SIM_SLOWDOWN_SPEED);
    ros::param::get("/SIM_SETTINGS/Landing_Slowdown_Flag",LANDING_SLOWDOWN_FLAG);

    ros::param::get("/SAR_DC_SETTINGS/Logging_Rate",LOGGING_RATE);
    ros::param::get("/SAR_DC_SETTINGS/Console_Output",SHOW_CONSOLE);

    if(DATA_TYPE.compare("SIM") == 0)
    {
        ros::param::set("/use_sim_time",true);
    }
    else
    {
        ros::param::set("/use_sim_time",false);
    }

}

inline bool SAR_DataConverter::CMD_SAR_DC_Callback(sar_msgs::CTRL_Cmd_srv::Request &req, sar_msgs::CTRL_Cmd_srv::Response &res)
{
    // PASS COMMAND VALUES TO CONTROLLER AND PASS LOCAL ACTIONS
    SAR_DataConverter::Send_Cmd2Ctrl(req);
    res.srv_Success = true;
    return res.srv_Success;
}

inline bool SAR_DataConverter::Send_Cmd2Ctrl(sar_msgs::CTRL_Cmd_srv::Request &req)
{
    switch (req.cmd_type)
    {
        case 0:
            // RESET FLIP TIME
            OnceFlag_flip = false;
            Time_tr.sec = 0.0;
            Time_tr.nsec = 0.0;
            Rot_Sum = 0.0;

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

            Pad_Connect_Sum = 0;

            if (DATA_TYPE.compare("SIM") == 0)
            {
                // RESET SIM SPEED
                SAR_DataConverter::adjustSimSpeed(SIM_SPEED);
                SLOWDOWN_TYPE = 0;
            }
            break;

        case 21:  // UPDATE PARAMS IN SAR_DC 
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
    sar_msgs::CTRL_Cmd_srv srv;
    srv.request = req;
    CMD_Output_Service.call(srv);


    // EXPERIMENT: 
    // SEND COMMAND VALUES TO PHYSICAL CONTROLLER
    // BROADCAST CMD VALUES AS ROS MESSAGE
    sar_msgs::CTRL_Cmd cmd_msg;
    cmd_msg.cmd_type = req.cmd_type;
    cmd_msg.cmd_vals = req.cmd_vals;
    cmd_msg.cmd_flag = req.cmd_flag;
    cmd_msg.cmd_rx = req.cmd_rx;

    for (int i = 0; i < 3; i++)
    {
        CMD_Output_Topic.publish(cmd_msg);
    }
    
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

