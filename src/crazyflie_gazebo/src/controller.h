#include <iostream>
#include <thread>
#include <Eigen/Dense>
#include <cmath>        // std::abs
#include <math.h>       


// ROS Includes
#include <ros/ros.h>
#include "crazyflie_gazebo/CtrlData.h"
#include "crazyflie_rl/RLCmd.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"

// Socket Includes
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>

#include "readerwriterqueue.h"

typedef struct _MotorCommand {
    float data[4];
} MotorCommand;


class Controller
{
    public:
        // CONSTRUCTOR TO START PUBLISHERS AND SUBSCRIBERS (Similar to Python's __init__() )
        Controller(ros::NodeHandle *nh){
            ctrl_Publisher = nh->advertise<crazyflie_gazebo::CtrlData>("/ctrl_data",1);

            // NOTE: tcpNoDelay() removes delay where system is waiting for datapackets to be fully filled before sending;
            // instead of sending data as soon as it is available to match publishing rate (This is an issue with large messages like Odom or Custom)
            globalState_Subscriber = nh->subscribe("/global_state",1,&Controller::global_stateCallback,this,ros::TransportHints().tcpNoDelay());
            OF_Subscriber = nh->subscribe("/OF_sensor",1,&Controller::OFCallback,this,ros::TransportHints().tcpNoDelay()); 
            imu_Subscriber = nh->subscribe("/imu",1,&Controller::imuCallback,this);
            // Queue lengths are set to '1' so only the newest data is used
            
            RLCmd_Subscriber = nh->subscribe("/rl_ctrl",50,&Controller::RLCmd_Callback,this);

            

            
            // INIT VARIABLES TO DEFAULT VALUES (PREVENTS RANDOM VALUES FROM MEMORY)
            _ctrl_cmd << 404,0,0,0,0;

            _t = 0.0; 
            _pos << 0,0,0;
            _vel << 0,0,0;
            _quat << 1,0,0,0;
            _omega << 0,0,0;

            _RREV = 0.0;
            _OF_x = 0.0;
            _OF_y = 0.0;


            // SET DEFAULT HOME POSITION
            _x_d << 0,0,0.6;
            _v_d << 0,0,0;
            _a_d << 0,0,0;
            _b1_d << 1,0,0;
            _omega_d << 0,0,0;

            // SET DEFAULT CONTROLLER GAINS


            _kp_x_D.fill(0.5);
            _kd_x_D.fill(0.15);
            _kp_R_D.fill(0.015);
            _kd_R_D.fill(0.0012);

            _kp_x = _kp_x_D;
            _kd_x = _kd_x_D;
            _kp_R = _kp_R_D;
            _kd_R = _kd_R_D;



            // SET DEFAULT POLICY VALUES
            _RREV_thr = 0.0;
            _G1 = 0.0;
            _G2 = 0.0;
            _policy_armed_flag = false;
            _flip_flag = false;
        }

        // DEFINE FUNCTION PROTOTYPES
        void Load();
        void recvThread_RL();
        void controlThread();
        void global_stateCallback(const nav_msgs::Odometry::ConstPtr &msg);
        void OFCallback(const nav_msgs::Odometry::ConstPtr &msg);
        void imuCallback(const sensor_msgs::Imu::ConstPtr &msg);
        void RLCmd_Callback(const crazyflie_rl::RLCmd::ConstPtr &msg);

    private:
        // DEFINE PUBLISHERS AND SUBSCRIBERS
        ros::Publisher ctrl_Publisher;
        ros::Subscriber globalState_Subscriber;
        ros::Subscriber OF_Subscriber;
        ros::Subscriber RLCmd_Subscriber;
        ros::Subscriber imu_Subscriber;

        // DEFINE THREAD OBJECTS
        std::thread controllerThread;
        std::thread senderThread_gazebo;

        

        // DEFINE CLASS VARIABLES (Similar to Python's class variables)
        // Leading '_' represents a class variable that works across functions
        bool _isRunning;
        Eigen::Matrix<double,5,1> _ctrl_cmd;
       

        double _t;
        Eigen::Vector3d _pos;   // Current position [m]
        Eigen::Vector3d _vel;   // Current velocity [m]
        Eigen::Vector4d _quat;  // Current attitude [rad] (quat form)
        Eigen::Vector3d _omega; // Current angular velocity [rad/s]


        Eigen::Vector3d _x_d;       // Pos-desired [m]  
        Eigen::Vector3d _v_d;       // Velocity-desired [m/s]
        Eigen::Vector3d _a_d;       // Acceleration-desired [m/s]
        Eigen::Vector3d _b1_d;      // Desired body-fixed x-axis in terms of global axes
        Eigen::Vector3d _omega_d;   // Omega-desired [rad/s]
        Eigen::Vector3d _M_d;       // Moment-desired [N*m]

        Eigen::Vector3d _eul_d;     // Euler-desired [rad?]
        bool _eul_flag = false;
        Eigen::Matrix3f _R_d_custom; // Rotation-desired (YZX Euler notation)

        Eigen::Vector3d _kp_x; // Pos. Gain
        Eigen::Vector3d _kd_x; // Pos. derivative Gain
        Eigen::Vector3d _kp_R; // Rot. Gain
        Eigen::Vector3d _kd_R; // Rot. derivative Gain

        Eigen::Vector3d _kp_x_D; // Pos. Gain
        Eigen::Vector3d _kd_x_D; // Pos. derivative Gain
        Eigen::Vector3d _kp_R_D; // Rot. Gain
        Eigen::Vector3d _kd_R_D; // Rot. derivative Gain


        double _RREV;
        double _OF_x;
        double _OF_y; 

        // POLICY FLAGS AND VALUES
        double _RREV_thr;
        double _G1;
        double _G2;
        bool _policy_armed_flag;
        bool _flip_flag;

        

        // CONTROLLER FLAGS
        double _kp_xf = 1; // Pos. Gain Flag
        double _kd_xf = 1; // Pos. derivative Gain Flag
        double _kp_Rf = 1; // Rot. Gain Flag
        double _kd_Rf = 1; // Rot. derivative Gain Flag

        bool _motorstop_flag = false;
        bool _Moment_flag = false;
        bool _tumbled = false;
        bool _tumble_detection = true;



        // DEFINE CTRL_MAVLINK SOCKET VARIABLES
        int Ctrl_Mavlink_socket;
        int Ctrl_Mavlink_socket_SNDBUF;
        int Ctrl_Mavlink_socket_RCVBUF;
        int Ctrl_Mavlink_socket_PORT;
        struct sockaddr_in addr_Ctrl_Mavlink;

        // DEFINE CTRL_RL SOCKET VARIABLES
        int Ctrl_RL_socket;
        int Ctrl_RL_socket_SNDBUF;
        int Ctrl_RL_socket_RCVBUF;
        int Ctrl_RL_socket_Port;
        struct sockaddr_in addr_Ctrl_RL;

        // DEFINE MAVLINK ADDRESS VARIABLES
        int Mavlink_PORT;
        struct sockaddr_in addr_Mavlink;
        socklen_t addr_Mavlink_len;

        // QUEUEING STUFF (I don't understand it yet)
        moodycamel::BlockingReaderWriterQueue<MotorCommand> queue_motorspeed;




};

// CUSTOM EIGEN FUNCTIONS FOR HAD AND DEHAT OPERATORS
Eigen::Matrix3d hat(Eigen::Vector3d a) // Input a hat vector and output corresponding skew-symmetric matrix
{ 
  // You hat a vector and get a skew-symmetric matrix
  // You dehat/dehat a skew-symmetric matrix and get a vector

    /* Convert a into skew symmetric matrix a_hat
    a = [ a1 ] 
        [ a2 ] 
        [ a3 ]
 
    a_hat = [  0   -a3   a2 ]
            [  a3   0   -a1 ]
            [ -a2   a1   0  ]
    ]
    */
    Eigen::Matrix3d a_hat;
    a_hat(2,1) =  a(0);
    a_hat(1,2) = -a(0);

    a_hat(0,2) =  a(1);
    a_hat(2,0) = -a(1);

    a_hat(1,0) =  a(2);
    a_hat(0,1) = -a(2);

    return a_hat;
}

Eigen::Vector3d dehat(Eigen::Matrix3d a_hat) // Input a skew-symmetric matrix and output corresponding vector
{

    /* Convert skew-symmetric matrix a_hat into vector a

    a_hat = [  0   -a3   a2 ]
            [  a3   0   -a1 ]
            [ -a2   a1   0  ]


    a = [ a1 ] 
        [ a2 ] 
        [ a3 ]

    */

    Eigen::Vector3d a;
    Eigen::Matrix3d tmp;

    tmp = (a_hat - a_hat.transpose())/2; // Not sure why this is done

    a(0) = tmp(2,1);
    a(1) = tmp(0,2);
    a(2) = tmp(1,0);

    return a;
}

static inline float clamp(float value, float min, float max) {
  if (value < min) return min;
  if (value > max) return max;
  return value;
}

static inline float limitThrust(float thrust)
{
    if(thrust < 0){
        thrust = 0;
    }
    else if(thrust >= 65535){ // Max rotation speed (rad/s)
        thrust = 65535;
    }

    return thrust;
}

static inline int32_t thrust2PWM(float f) // Converts thrust in Newtons to PWM
{
    
    // Conversion values calculated from self motor analysis
    float a = 7.77e-11;
    float b = 2.64e-4;

    float s = 1; // sign of value
    int32_t f_pwm = 0;

    s = f/fabsf(f);
    f = fabsf(f)*1000.0f/9.81f; // Convert thrust to grams
    
    f_pwm = s*(sqrtf(4*a*f+b*b)/(2*a) - b/(2*a));

    return f_pwm;
}

static inline float PWM2thrust(int32_t M_PWM) // Converts thrust in PWM to thrust in Newtons
{
    // Conversion values from new motors
    float a = 7.77e-11; 
    float b = 2.64e-4;

    float kf = 2.2e-8; // Thrust constant [N/(rad/s)^2]

    float f = (a*M_PWM*M_PWM + b*M_PWM); // Convert thrust to grams

    f = f*9.81/1000; // Convert thrust from grams to Newtons

    return f;
}