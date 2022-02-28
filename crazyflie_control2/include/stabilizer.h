
#include <iostream>
#include <thread>
#include <cmath>        // std::abs
#include <stdio.h>
#include <string>


#include "stabilizer_types.h"
#include "controller_gtc.h"
#include "estimator.h"
#include "nml.h"


#include <ros/ros.h>
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "crazyflie_msgs/OF_SensorData.h"
#include "crazyflie_msgs/MS.h"

#include "crazyflie_msgs/CtrlData.h"
#include "crazyflie_msgs/ImpactData.h"
#include "crazyflie_msgs/RLCmd.h"
#include "crazyflie_msgs/RLData.h"
#include "crazyflie_msgs/PadConnect.h"
#include "crazyflie_msgs/MS.h"

#include "gazebo_msgs/SetPhysicsProperties.h"



uint32_t tick;

// State variables for the stabilizer
static setpoint_t setpoint;
static sensorData_t sensorData;
static state_t state;
static control_t control;

class Controller
{
    public:

        Controller(ros::NodeHandle *nh)
        {
            // SUBSCRIBERS
            Vicon_Subscriber = nh->subscribe("/UKF/viconState_Filtered",1,&Controller::viconState_Callback,this,ros::TransportHints().tcpNoDelay());
            IMU_Subscriber = nh->subscribe("/CF_Internal/IMU",1,&Controller::imuState_Callback,this,ros::TransportHints().tcpNoDelay());
            OF_Subscriber = nh->subscribe("/CF_Internal/OF_Sensor",1,&Controller::OFState_Callback,this,ros::TransportHints().tcpNoDelay());
            CMD_Subscriber = nh->subscribe("/rl_ctrl",50,&Controller::CMD_Callback,this,ros::TransportHints().tcpNoDelay());
            CeilingFT_Subcriber = nh->subscribe("/env/ceiling_force_sensor",5,&Controller::ceilingFT_Callback,this,ros::TransportHints().tcpNoDelay());
            RLData_Subscriber = nh->subscribe("/rl_data",5,&Controller::RLData_Callback,this,ros::TransportHints().tcpNoDelay());

            // PUBLISHERS
            MS_PWM_Publisher = nh->advertise<crazyflie_msgs::MS>("/MS",1);
            CTRL_Publisher = nh->advertise<crazyflie_msgs::CtrlData>("/ctrl_data",1);



            // SERVICES
            _SimSpeed_Client = nh->serviceClient<gazebo_msgs::SetPhysicsProperties>("/gazebo/set_physics_properties");

            
            Controller::loadParams();
            Controller::adjustSimSpeed(_SIM_SPEED);
            controllerThread = std::thread(&Controller::stabilizerLoop, this);
        }

        

        // SUBSCRIBERS
        ros::Subscriber Vicon_Subscriber;
        ros::Subscriber IMU_Subscriber;
        ros::Subscriber OF_Subscriber;

        ros::Subscriber CMD_Subscriber;
        ros::Subscriber CeilingFT_Subcriber;
        ros::Subscriber RLData_Subscriber;

        // PUBLISHERS
        ros::Publisher MS_PWM_Publisher;
        ros::Publisher CTRL_Publisher;

        // SERVICES
        ros::ServiceClient _SimSpeed_Client;

        // DEFINE THREAD OBJECTS
        std::thread controllerThread;

        uint32_t tick;
        ros::Time t;

        // ROS SPECIFIC VALUES
        int _impact_flag = 0;
        int _slowdown_type = 0;
        float _H_CEILING = 2.10;
        bool _LANDING_SLOWDOWN_FLAG;
        float _SIM_SPEED; 
        float _SIM_SLOWDOWN_SPEED;
        float _CF_MASS;
        int _POLICY_TYPE;
        std::string _MODEL_NAME;

        // FUNCTION PRIMITIVES
        void viconState_Callback(const nav_msgs::Odometry::ConstPtr &msg);
        void imuState_Callback(const sensor_msgs::Imu::ConstPtr &msg);
        void OFState_Callback(const crazyflie_msgs::OF_SensorData::ConstPtr &msg);

        void CMD_Callback(const crazyflie_msgs::RLCmd::ConstPtr &msg);
        void ceilingFT_Callback(const crazyflie_msgs::ImpactData::ConstPtr &msg);
        void RLData_Callback(const crazyflie_msgs::RLData::ConstPtr &msg);

        void stabilizerLoop();
        void loadParams();
        void adjustSimSpeed(float speed_mult);
        void consoleOuput();
        void checkSlowdown();
        

        crazyflie_msgs::MS MS_msg;
        crazyflie_msgs::CtrlData CTRL_msg;

        gazebo_msgs::SetPhysicsProperties srv;


};




void Controller::OFState_Callback(const crazyflie_msgs::OF_SensorData::ConstPtr &msg)
{
    sensorData.Tau = msg->Tau;
    sensorData.OFx = msg->OFx;
    sensorData.OFy = msg->OFy;
    sensorData.RREV = msg->RREV;

}

void Controller::imuState_Callback(const sensor_msgs::Imu::ConstPtr &msg)
{
    sensorData.acc.x = msg->linear_acceleration.x/9.8066; // Convert to Gs to match crazyflie sensors
    sensorData.acc.y = msg->linear_acceleration.y/9.8066;
    sensorData.acc.z = msg->linear_acceleration.z/9.8066;

}

void Controller::viconState_Callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    
    // UPDATE POSE FROM VICON SYSTEM
    state.position.x = msg->pose.pose.position.x;
    state.position.y = msg->pose.pose.position.y;
    state.position.z = msg->pose.pose.position.z;

    state.attitudeQuaternion.x = msg->pose.pose.orientation.x;
    state.attitudeQuaternion.y = msg->pose.pose.orientation.y;
    state.attitudeQuaternion.z = msg->pose.pose.orientation.z;
    state.attitudeQuaternion.w = msg->pose.pose.orientation.w;

    // UPDATE VELOCITIES FROM VICON SYSTEM
    state.velocity.x = msg->twist.twist.linear.x;
    state.velocity.y = msg->twist.twist.linear.y;
    state.velocity.z = msg->twist.twist.linear.z;

    sensorData.gyro.x = msg->twist.twist.angular.x*180.0/M_PI; // Convert to deg/s to match crazyflie sensors
    sensorData.gyro.y = msg->twist.twist.angular.y*180.0/M_PI;
    sensorData.gyro.z = msg->twist.twist.angular.z*180.0/M_PI;

}

void Controller::CMD_Callback(const crazyflie_msgs::RLCmd::ConstPtr &msg)
{
    setpoint.cmd_type = msg->cmd_type;
    setpoint.cmd_val1 = msg->cmd_vals.x;
    setpoint.cmd_val2 = msg->cmd_vals.y;
    setpoint.cmd_val3 = msg->cmd_vals.z;
    setpoint.cmd_flag = msg->cmd_flag;

    setpoint.GTC_cmd_rec = true;

    if(msg->cmd_type == 6)
    {
        Controller::loadParams();
    }
}

void Controller::ceilingFT_Callback(const crazyflie_msgs::ImpactData::ConstPtr &msg)
{
    _impact_flag = msg->impact_flag;
}

void Controller::RLData_Callback(const crazyflie_msgs::RLData::ConstPtr &msg){

    if (msg->reset_flag == true){

        controllerGTCReset();

    }
}


void Controller::loadParams()
{
    // SIMULATION SETTINGS FROM CONFIG FILE
    ros::param::get("/MODEL_NAME",_MODEL_NAME);
    ros::param::get("/CEILING_HEIGHT",_H_CEILING);
    ros::param::get("/CF_MASS",_CF_MASS);
    ros::param::get("/POLICY_TYPE",_POLICY_TYPE);
    POLICY_TYPE = (Policy_Type)_POLICY_TYPE; // Cast ROS param (int) to enum (Policy_Type)

    // DEBUG SETTINGS
    ros::param::get("/SIM_SPEED",_SIM_SPEED);
    ros::param::get("/SIM_SLOWDOWN_SPEED",_SIM_SLOWDOWN_SPEED);
    ros::param::get("/LANDING_SLOWDOWN_FLAG",_LANDING_SLOWDOWN_FLAG);

    // COLLECT CTRL GAINS FROM CONFIG FILE
    ros::param::get("P_kp_xy",P_kp_xy);
    ros::param::get("P_kd_xy",P_kd_xy);
    ros::param::get("P_ki_xy",P_ki_xy);
    ros::param::get("i_range_xy",i_range_xy);

    ros::param::get("P_kp_z",P_kp_z);
    ros::param::get("P_kd_z",P_kd_z);
    ros::param::get("P_ki_z",P_ki_z);
    ros::param::get("i_range_z",i_range_z);

    ros::param::get("R_kp_xy",R_kp_xy);
    ros::param::get("R_kd_xy",R_kd_xy);
    ros::param::get("R_ki_xy",R_ki_xy);
    ros::param::get("i_range_R_xy",i_range_R_xy);
    
    ros::param::get("R_kp_z",R_kp_z);
    ros::param::get("R_kd_z",R_kd_z);
    ros::param::get("R_ki_z",R_ki_z);
    ros::param::get("i_range_R_z",i_range_R_z);

}

void Controller::adjustSimSpeed(float speed_mult)
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

    _SimSpeed_Client.call(srv);
}

void Controller::consoleOuput()
{
    system("clear");
    printf("t: %.4f \tCmd: \n",ros::Time::now().toSec());
    printf("Model: %s\n",_MODEL_NAME.c_str());
    printf("\n");

    printf("==== Flags ====\n");
    printf("Policy_armed:\t %u  Slowdown_type:\t %u  kp_xf:\t %u \n",policy_armed_flag,_slowdown_type,(int)kp_xf);
    printf("Flip_flag:\t %u  Impact_flag:\t %u  kd_xf:\t %u \n",flip_flag,_impact_flag,(int)kd_xf);
    printf("Tumbled:\t %u  Tumble Detect:\t %u  Traj Active: %u \n",tumbled,tumble_detection,execute_traj);
    printf("Motorstop:\t %u  Policy_type:\t %u\n",motorstop_flag,POLICY_TYPE);
    printf("\n");

    printf("==== Setpoints ====\n");
    printf("x_d: %.3f  %.3f  %.3f\n",x_d.x,x_d.y,x_d.z);
    printf("v_d: %.3f  %.3f  %.3f\n",v_d.x,v_d.y,v_d.z);
    printf("a_d: %.3f  %.3f  %.3f\n",a_d.x,a_d.y,a_d.z);
    printf("\n");

    printf("==== System States ====\n");
    printf("Pos [m]:\t %.3f  %.3f  %.3f\n",statePos.x,statePos.y,statePos.z);
    printf("Vel [m/s]:\t %.3f  %.3f  %.3f\n",stateVel.x,stateVel.y,stateVel.z);
    printf("Omega [rad/s]:\t %.3f  %.3f  %.3f\n",stateOmega.x,stateOmega.y,stateOmega.z);
    printf("Eul [deg]:\t %.3f  %.3f  %.3f\n",stateEul.x,stateEul.y,stateEul.z);
    printf("\n");

    printf("Tau: %.3f \tOFx: %.3f \tOFy: %.3f \tRREV: %.3f\n",Tau,OFx,OFy,RREV);
    printf("D_ceil: %.3f\n",d_ceil);
    printf("\n");

    printf("==== Policy Values ====\n");
    printf("RL: \n");
    printf("RREV_thr: %.3f \tG1: %.3f \tG2: %.3f\n",RREV_thr,G1,G2);
    printf("\n");

    printf("NN_Outputs: \n");
    printf("NN_Flip:  %.3f \tNN_Policy: %.3f \n",NN_flip,NN_policy);
    printf("\n");

    printf("==== Flip Trigger Values ====\n");
    printf("RREV_tr:    %.3f \tNN_tr_Flip:    %.3f \n",RREV_tr,NN_tr_flip);
    printf("OFy_tr:     %.3f \tNN_tr_Policy:  %.3f \n",OFy_tr,NN_tr_policy);
    printf("D_ceil_tr:  %.3f \n",d_ceil_tr);
    printf("\n");

    printf("==== Controller Actions ====\n");
    printf("FM [N/N*mm]: %.3f  %.3f  %.3f  %.3f\n",F_thrust,M.x*1.0e3,M.y*1.0e3,M.z*1.0e3);
    printf("f [g]: %.3f  %.3f  %.3f  %.3f\n",f_thrust_g,f_roll_g,f_pitch_g,f_yaw_g);
    printf("\n");

    printf("MS_PWM: %u  %u  %u  %u\n",M1_pwm,M2_pwm,M3_pwm,M4_pwm);
    printf("\n");


    printf("=== Parameters ====\n");
    printf("Kp_P: %.3f  %.3f  %.3f \t",Kp_p.x,Kp_p.y,Kp_p.z);
    printf("Kp_R: %.3f  %.3f  %.3f \n",Kp_R.x,Kp_R.y,Kp_R.z);
    printf("Kd_P: %.3f  %.3f  %.3f \t",Kd_p.x,Kd_p.y,Kd_p.z);
    printf("Kd_R: %.3f  %.3f  %.3f \n",Kd_R.x,Kd_R.y,Kd_R.z);
    printf("Ki_P: %.3f  %.3f  %.3f \t",Ki_p.x,Ki_p.y,Ki_p.z);
    printf("Ki_R: %.3f  %.3f  %.3f \n",Ki_p.x,Ki_p.y,Ki_p.z);
    printf("======\n");
}

void Controller::checkSlowdown()
{   
    // SIMULATION SLOWDOWN
    if(_LANDING_SLOWDOWN_FLAG==true && tick >= 500){

        // WHEN CLOSE TO THE CEILING REDUCE SIM SPEED
        if(d_ceil<=0.5 && _slowdown_type == 0){
            
            Controller::adjustSimSpeed(_SIM_SLOWDOWN_SPEED);
            _slowdown_type = 1;
        }

        // IF IMPACTED CEILING OR FALLING AWAY, INCREASE SIM SPEED TO DEFAULT
        if(_impact_flag == true && _slowdown_type == 1)
        {
            Controller::adjustSimSpeed(_SIM_SPEED);
            _slowdown_type = 2;
        }
        else if(stateVel.z <= -0.5 && _slowdown_type == 1){
            Controller::adjustSimSpeed(_SIM_SPEED);
            _slowdown_type = 2;
        }
    
    }

}