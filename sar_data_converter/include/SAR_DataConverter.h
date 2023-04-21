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

            GZ_SimSpeed_Client = nh->serviceClient<gazebo_msgs::SetPhysicsProperties>("/gazebo/set_physics_properties");


            SAR_DC_Thread = std::thread(&SAR_DataConverter::MainLoop, this);
            // ConsoleOutput_Thread = std::thread(&SAR_DataConverter::ConsoleLoop, this);


        }

        void MainInit();
        void MainLoop();
        void ConsoleLoop();


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
        bool CMD_CF_DC_Callback(crazyflie_msgs::GTC_Cmd_srv::Request &req, crazyflie_msgs::GTC_Cmd_srv::Response &res);
        bool CMD_Dashboard_Callback(crazyflie_msgs::GTC_Cmd_srv::Request &req, crazyflie_msgs::GTC_Cmd_srv::Response &res);
        bool Send_Cmd2Ctrl(crazyflie_msgs::GTC_Cmd_srv::Request &req);


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
        void Publish_StateData();
        void Publish_FlipData();
        void Publish_ImpactData();
        void Publish_MiscData();


        // =======================
        //     MISC. FUNCTIONS
        // =======================
        void quat2euler(float quat[], float eul[]);
        void euler2quat(float quat[],float eul[]);
        void LoadParams();


        

    private:
    
        std::thread SAR_DC_Thread;
        std::thread ConsoleOutput_Thread;

        // =====================
        //     SYSTEM PARAMS
        // =====================
        std::string DATA_TYPE;  // Sim or Experiment Flag
        uint32_t tick = 0;      // Tick for each loop iteration
        ros::Time Time_start;   // Initial time in UNIX notation
        int LOGGING_RATE = 25; // Default Logging Rate


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
        geometry_msgs::Vector3 Plane_Pos_0; // Initial Plane Position
        float Plane_Angle_0 = 180.0; // Initial Plane Angle [Deg]

        


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

        ros::ServiceClient Landing_Surface_Pose_Service;
        ros::ServiceClient GZ_SimSpeed_Client;

        // ===========================
        //     GTC COMMAND OBJECTS
        // ===========================
        ros::ServiceServer CMD_Service_CF_DC;
        ros::ServiceServer CMD_Service_Dashboard;
        ros::ServiceClient CMD_Client;
        ros::Publisher CMD_Pub;

        // ============================
        //     DATA PUBLISH OBJECTS
        // ============================
        ros::Publisher StateData_Pub;
        ros::Publisher FlipData_Pub;
        ros::Publisher ImpactData_Pub;
        ros::Publisher MiscData_Pub;

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
        FILE* fPtr; // File Pointer to logging file
        bool Logging_Flag = false;
        std::string error_string = "No_Data";

    
};

