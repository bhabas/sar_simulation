/* 
This script is the main controller loop for the crazyflie. It receives
system and sensor states from the crazyflie model/gazebo via ROS topics
which it then passes to the Geometric Tracking Controller (controller_gtc.c).
It then outputs all of the chosen values via ROS topics to the 
Crazyflie_DataConverter (CF_DC) which collates all data from multiple nodes, 
reorganizes it, then republishes it. 

All changes to controller_gtc.c should remain in terms of C so it can easily
be transferred to the Crazyflie Firmware.
*/


// C++ Includes
#include <iostream>
#include <thread>
#include <stdio.h>
#include <string>

// "Firmware" Includes
#include "stabilizer_types.h"
#include "controller_gtc.h"
#include "estimator.h"
#include "nml.h"


#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include "sensor_msgs/Image.h"


#include "crazyflie_msgs/OF_SensorData.h"
#include "crazyflie_msgs/MS.h"

#include "crazyflie_msgs/CtrlData.h"
#include "crazyflie_msgs/CtrlDebug.h"

#include "crazyflie_msgs/RLCmd.h"
#include "crazyflie_msgs/RLData.h"



// FIRMWARE VARIABLES FOR CONTROLLER
static setpoint_t setpoint;
static sensorData_t sensorData;
static state_t state;
static control_t control;

class Controller
{
    public:

        Controller(ros::NodeHandle *nh)
        {
            // ===========================
            //     ROS TOPICS/SERVICES
            // ===========================

            // CONTROLLER TOPICS
            CTRL_Data_Publisher = nh->advertise<crazyflie_msgs::CtrlData>("/CTRL/data",1);
            CTRL_Debug_Publisher = nh->advertise<crazyflie_msgs::CtrlDebug>("CTRL/debug",1);

            // RL TOPICS
            RL_CMD_Subscriber = nh->subscribe("/RL/cmd",5,&Controller::RL_CMD_Callback,this,ros::TransportHints().tcpNoDelay());

            // INTERNAL TOPICS
            CF_IMU_Subscriber = nh->subscribe("/CF_Internal/IMU",1,&Controller::IMU_Sensor_Callback,this,ros::TransportHints().tcpNoDelay());
            CF_OF_Subscriber = nh->subscribe("/CF_Internal/OF_Sensor",1,&Controller::OF_Sensor_Callback,this,ros::TransportHints().tcpNoDelay());
            CF_Camera_Subscriber = nh->subscribe("/CF_Internal/camera/image_raw",1,&Controller::Camera_Sensor_Callback,this,ros::TransportHints().tcpNoDelay());

            // ENVIRONMENT TOPICS
            ENV_Vicon_Subscriber = nh->subscribe("/ENV/viconState_UKF",1,&Controller::viconState_Callback,this,ros::TransportHints().tcpNoDelay());
            
            // DYNAMICALLY ALLOCATE MEMORY TO STORE PREVIOUS IMAGE
            Prev_img = (uint8_t*)calloc(WIDTH_PIXELS*HEIGHT_PIXELS,sizeof(uint8_t));



            Controller::loadParams();

            // Thread main controller loop so other callbacks can work fine
            controllerThread = std::thread(&Controller::stabilizerLoop, this);
        }

        

        // SUBSCRIBERS
        ros::Subscriber CF_IMU_Subscriber;
        ros::Subscriber CF_OF_Subscriber;
        ros::Subscriber CF_Camera_Subscriber;

        ros::Subscriber RL_CMD_Subscriber;
        ros::Subscriber RL_Data_Subscriber;

        ros::Subscriber ENV_Vicon_Subscriber;
        ros::Subscriber ENV_CeilingFT_Subscriber;

        // PUBLISHERS
        ros::Publisher CF_PWM_Publisher;
        ros::Publisher CTRL_Data_Publisher;
        ros::Publisher CTRL_Debug_Publisher;

        // SERVICES
        ros::ServiceClient GZ_SimSpeed_Client;

        // DEFINE THREAD OBJECTS
        std::thread controllerThread;

        uint32_t tick = 1;

        // ROS SPECIFIC VALUES
        int _slowdown_type = 0;
        float _H_CEILING = 2.10;
        bool _LANDING_SLOWDOWN_FLAG;
        float _SIM_SPEED; 
        float _SIM_SLOWDOWN_SPEED;
        float _CF_MASS;
        int _POLICY_TYPE;
        std::string _MODEL_NAME;
        bool STICKY_FLAG = false;

        // IMAGE PROCESSING VARIABLES
        uint8_t WIDTH_PIXELS = 160;
        uint8_t HEIGHT_PIXELS = 160;
        int kx0[3] = {-1, 0, 1};
        int kx1[3] = {-2, 0, 2};
        int kx2[3] = {-1, 0, 1};
        int ky0[3] = {-1,-2,-1};
        int ky2[3] = {1,2,1};
        float Prev_time = 1; //init as 1 to prevent divide by zero for first image

        const uint8_t* Cur_img = NULL;  // Ptr to current image memory
        uint8_t* Prev_img = NULL;       // Ptr to prev image memory


        // FUNCTION PRIMITIVES
        void viconState_Callback(const nav_msgs::Odometry::ConstPtr &msg);
        void IMU_Sensor_Callback(const sensor_msgs::Imu::ConstPtr &msg);
        void OF_Sensor_Callback(const crazyflie_msgs::OF_SensorData::ConstPtr &msg);
        void Camera_Sensor_Callback(const sensor_msgs::Image::ConstPtr &msg);
        void RL_CMD_Callback(const crazyflie_msgs::RLCmd::ConstPtr &msg);

        void stabilizerLoop();
        void loadParams();
        void consoleOuput();
        void publishCtrlData();
        void publishCtrlDebug();
        

        crazyflie_msgs::CtrlData CtrlData_msg;
        crazyflie_msgs::CtrlDebug CtrlDebug_msg;

};

void Controller::Camera_Sensor_Callback(const sensor_msgs::Image::ConstPtr &msg)
{
    Cur_img = &(msg->data)[0]; // Point to current image data address

    //Where the convolution starts
    uint16_t X = 1;
    uint16_t Y = 1;
    float w = 3.6e-6; //Pixel width in meters
    float f = 0.33e-3;//Focal length in meters
    float U;
    float V;
    float O_up = WIDTH_PIXELS/2;
    float V_up = WIDTH_PIXELS/2;
    float Gtemp = 0;
    float Iuu = 0;
    float Ivv = 0;
    float Iuv = 0;
    float IGu = 0;
    float IGv = 0;
    float IGG = 0;
    float Iut = 0;
    float Ivt = 0;
    float IGt = 0;
    float dt;
    int16_t Ittemp;
    float Cur_time = ros::Time::now().toSec();
    
    for(int j = 0; j < (WIDTH_PIXELS - 2)*(HEIGHT_PIXELS - 2); j++) // How many times the kernel center moves around the image
    {

        //GENERALIZE FOR CHANGE IN KERNEL SIZE
        if(X >= WIDTH_PIXELS - 1) //if the edge of the kernel hits the edge of the image
        { 
        
            X = 1; //move the kernel back to the left edge of the image
            Y++; //and slide the kernel down the image

        }

        //Sub Kernel Indexing 
        uint16_t i0 = (X - 1) + (Y - 1) * WIDTH_PIXELS; //First grab top left location of whole kernel
        uint16_t i1 = i0 + WIDTH_PIXELS; //then each following row is separated by the image width
        uint16_t i2 = i1 + WIDTH_PIXELS;

        U = (X - O_up)*w + (w/2); // Using current location of the Kernel center
        V = (Y - V_up)*w + (w/2); //calculate the current pixel grid locations (u,v)

        // ######  DEBUGGING  ######
        /*//
        std::cout << "i0: " << i0 << "\n";
        std::cout << "i1: " << i1 << "\n";
        std::cout << "i2: " << i2 << "\n";
        *///

        int Xsum = 0; //reset rolling sum to 0
        int Ysum = 0;

        //GENERALIZE FOR CHANGE IN KERNEL SIZE
        for(int k = 0; k < 3; k++){

            //Sub kernel 0
            Xsum += kx0[k] * Cur_img[i0 + k];
            Ysum += ky0[k] * Cur_img[i0 + k];

            //Sub kernel 1 (skipping ky1)
            Xsum += kx1[k] * Cur_img[i1 + k];

            //Sub kernel 2
            Xsum += kx2[k] * Cur_img[i2 + k];
            Ysum += ky2[k] * Cur_img[i2 + k];

        }

        //Sum assigned to middle value: (i1 + 1)
        Ittemp = (Cur_img[i1 + 1] - Prev_img[i1 + 1]); //moved /dt to last step
        Gtemp = (Xsum*U + Ysum*V);

        //LHS Matrix values (rolling sums)
        Iuu += Xsum*Xsum;
        Ivv += Ysum*Ysum;
        Iuv += Xsum*Ysum;
        IGu += Gtemp*Xsum;
        IGv += Gtemp*Ysum;
        IGG += Gtemp*Gtemp;

        //RHS Matrix Values (rolling sums)
        Iut += Xsum*Ittemp;
        Ivt += Ysum*Ittemp; 
        IGt += Gtemp*Ittemp;

        X++; // move center of kernel over
        
    } // END OF CONVOLUTION

    dt = Cur_time - Prev_time;

    // Packing final result into the matrices and applying the floating point math
    double LHS[9] = {f/powf(8*w,2)*Iuu, f/powf(8*w,2)*Iuv, 1/powf(8*w,2)*IGu,
                     f/powf(8*w,2)*Iuv, f/powf(8*w,2)*Ivv, 1/powf(8*w,2)*IGv,
                     f/powf(8*w,2)*IGu, f/powf(8*w,2)*IGv, 1/powf(8*w,2)*IGG};

    double RHS[3] = {-Iut/(8*w*dt), -Ivt/(8*w*dt), -IGt/(8*w*dt)}; //added change in time to final step

    nml_mat* m_A = nml_mat_from(3,3,9,LHS);
    nml_mat* m_b = nml_mat_from(3,1,3,RHS);

    nml_mat_qr *QR = nml_mat_qr_solve(m_A); // A = Q*R
    nml_mat* y = nml_mat_dot(nml_mat_transp(QR->Q),m_b); // y = Q^T*b
    nml_mat* x_QR = nml_ls_solvebck(QR->R,y); // Solve R*x = y via back substitution
    nml_mat_print(x_QR);

    sensorData.OFx = x_QR->data[0][0];
    sensorData.OFy = x_QR->data[1][0];
    sensorData.Tau = 1/x_QR->data[2][0];
    sensorData.RREV = x_QR->data[2][0];


    nml_mat_free(m_A);
    nml_mat_free(m_b);
    nml_mat_qr_free(QR);
    nml_mat_free(x_QR);



    printf("Prev_Val: %u\n",Prev_img[0]);
    printf("Cur_Val: %u\n",Cur_img[0]);
    printf("\n");

    
    memcpy(Prev_img,Cur_img,sizeof(msg->data)); // Copy memory to Prev_img address
    Prev_time = Cur_time; // Setup previous time for next calculation

} // End of Camera_Sensor_Callback


// OPTICAL FLOW VALUES (IN BODY FRAME) FROM MODEL SENSOR PLUGIN
void Controller::OF_Sensor_Callback(const crazyflie_msgs::OF_SensorData::ConstPtr &msg)
{
    // sensorData.Tau = msg->Tau;
    // sensorData.OFx = msg->OFx;
    // sensorData.OFy = msg->OFy;
    // sensorData.RREV = msg->RREV;
    sensorData.d_ceil = msg->d_ceil;

}

// IMU VALUES FROM MODEL SENSOR PLUGIN
void Controller::IMU_Sensor_Callback(const sensor_msgs::Imu::ConstPtr &msg)
{
    sensorData.acc.x = msg->linear_acceleration.x/9.8066; // Convert to Gs to match crazyflie sensors
    sensorData.acc.y = msg->linear_acceleration.y/9.8066;
    sensorData.acc.z = msg->linear_acceleration.z/9.8066;

}

// POSE AND TWIST FROM "VICON" SYSTEM (GAZEBO WORLD FRAME)
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

// RECEIVE COMMANDS FROM RL SCRIPTS
void Controller::RL_CMD_Callback(const crazyflie_msgs::RLCmd::ConstPtr &msg)
{
    setpoint.cmd_type = msg->cmd_type;
    setpoint.cmd_val1 = msg->cmd_vals.x;
    setpoint.cmd_val2 = msg->cmd_vals.y;
    setpoint.cmd_val3 = msg->cmd_vals.z;
    setpoint.cmd_flag = msg->cmd_flag;

    setpoint.GTC_cmd_rec = true;

    if(msg->cmd_type == 6) // RESET ROS PARAM VALUES
    {
        Controller::loadParams();

    }

}


// LOAD VALUES FROM ROSPARAM SERVER INTO CONTROLLER
void Controller::loadParams()
{
    // COLLECT MODEL PARAMETERS
    ros::param::get("/CF_MASS",m);
    ros::param::get("/Ixx",Ixx);
    ros::param::get("/Iyy",Iyy);
    ros::param::get("/Izz",Izz);

    // SIMULATION SETTINGS FROM CONFIG FILE
    ros::param::get("/POLICY_TYPE",_POLICY_TYPE);

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


void Controller::publishCtrlDebug()
{
    CtrlDebug_msg.Motorstop_Flag = motorstop_flag;
    CtrlDebug_msg.Pos_Ctrl = (bool)kp_xf;
    CtrlDebug_msg.Vel_Ctrl = (bool)kd_xf;
    CtrlDebug_msg.Traj_Active = execute_traj;
    CtrlDebug_msg.Tumble_Detection = tumble_detection;
    CtrlDebug_msg.Tumbled_Flag = tumbled;
    CtrlDebug_msg.Moment_Flag = moment_flag; 
    CtrlDebug_msg.Policy_Armed = policy_armed_flag; 

    CTRL_Debug_Publisher.publish(CtrlDebug_msg);
}

// PUBLISH CONTROLLER DATA ON ROS TOPIC
void Controller::publishCtrlData()
{
    // STATE DATA
    CtrlData_msg.Pose.position.x = statePos.x;
    CtrlData_msg.Pose.position.y = statePos.y;
    CtrlData_msg.Pose.position.z = statePos.z;

    CtrlData_msg.Pose.orientation.x = stateQuat.x;
    CtrlData_msg.Pose.orientation.y = stateQuat.y;
    CtrlData_msg.Pose.orientation.z = stateQuat.z;
    CtrlData_msg.Pose.orientation.w = stateQuat.w;

    CtrlData_msg.Twist.linear.x = stateVel.x;
    CtrlData_msg.Twist.linear.y = stateVel.y;
    CtrlData_msg.Twist.linear.z = stateVel.z;

    CtrlData_msg.Twist.angular.x = stateOmega.x;
    CtrlData_msg.Twist.angular.y = stateOmega.y;
    CtrlData_msg.Twist.angular.z = stateOmega.z;

    // OPTICAL FLOW DATA
    CtrlData_msg.Tau = Tau;
    CtrlData_msg.OFx = OFx;
    CtrlData_msg.OFy = OFy;
    CtrlData_msg.RREV = RREV;
    CtrlData_msg.D_ceil = d_ceil;

    CtrlData_msg.Tau_thr = Tau_thr;
    CtrlData_msg.G1 = G1;

    // NEURAL NETWORK DATA
    CtrlData_msg.NN_policy = NN_policy;
    CtrlData_msg.NN_flip = NN_flip;

    // CONTROL ACTIONS
    CtrlData_msg.FM = {F_thrust,M.x*1.0e3,M.y*1.0e3,M.z*1.0e3};
    CtrlData_msg.MotorThrusts = {M1_thrust,M2_thrust,M3_thrust,M4_thrust};
    CtrlData_msg.MS_PWM = {M1_pwm,M2_pwm,M3_pwm,M4_pwm};

    CtrlData_msg.x_d.x = x_d.x;
    CtrlData_msg.x_d.y = x_d.y;
    CtrlData_msg.x_d.z = x_d.z;

    CtrlData_msg.v_d.x = v_d.x;
    CtrlData_msg.v_d.y = v_d.y;
    CtrlData_msg.v_d.z = v_d.z;

    CtrlData_msg.a_d.x = a_d.x;
    CtrlData_msg.a_d.y = a_d.y;
    CtrlData_msg.a_d.z = a_d.z;



    // STATE DATA (FLIP)
    CtrlData_msg.flip_flag = flip_flag;

    // CtrlData_msg.Pose_tr.header.stamp = t_flip;             
    CtrlData_msg.Pose_tr.position.x = statePos_tr.x;
    CtrlData_msg.Pose_tr.position.y = statePos_tr.y;
    CtrlData_msg.Pose_tr.position.z = statePos_tr.z;

    CtrlData_msg.Pose_tr.orientation.x = stateQuat_tr.x;
    CtrlData_msg.Pose_tr.orientation.y = stateQuat_tr.y;
    CtrlData_msg.Pose_tr.orientation.z = stateQuat_tr.z;
    CtrlData_msg.Pose_tr.orientation.w = stateQuat_tr.w;

    CtrlData_msg.Twist_tr.linear.x = stateVel_tr.x;
    CtrlData_msg.Twist_tr.linear.y = stateVel_tr.y;
    CtrlData_msg.Twist_tr.linear.z = stateVel_tr.z;

    CtrlData_msg.Twist_tr.angular.x = stateOmega_tr.x;
    CtrlData_msg.Twist_tr.angular.y = stateOmega_tr.y;
    CtrlData_msg.Twist_tr.angular.z = stateOmega_tr.z;

    // OPTICAL FLOW DATA (FLIP)
    CtrlData_msg.Tau_tr = Tau_tr;
    CtrlData_msg.OFx_tr = OFx_tr;
    CtrlData_msg.OFy_tr = OFy_tr;
    CtrlData_msg.RREV_tr = RREV_tr;
    CtrlData_msg.D_ceil_tr = d_ceil_tr;

    // NEURAL NETWORK DATA (FLIP)
    CtrlData_msg.NN_tr_flip = NN_tr_flip;
    CtrlData_msg.NN_tr_policy = NN_tr_policy;

    // CONTROL ACTIONS (FLIP)
    CtrlData_msg.FM_flip = {F_thrust_flip,M_x_flip*1.0e3,M_y_flip*1.0e3,M_z_flip*1.0e3};

    
    CTRL_Data_Publisher.publish(CtrlData_msg);

}
