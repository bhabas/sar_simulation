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
#include "gazebo_msgs/SetPhysicsProperties.h"

// Socket Includes
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>

#include "readerwriterqueue.h"

#define PWM_MAX 60000
#define f_MAX (16.5)
#define g2Newton (9.81f/1000.0f)
#define Newton2g (1000.0f/9.81f)

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

            client = nh->serviceClient<gazebo_msgs::SetPhysicsProperties>("/gazebo/set_physics_properties");



            
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
            _x_d << 0,0,0.4;
            _v_d << 0,0,0;
            _a_d << 0,0,0;
            _b1_d << 1,0,0;
            _omega_d << 0,0,0;

            // SET DEFAULT POLICY VALUES
            _RREV_thr = 0.0;
            _G1 = 0.0;
            _G2 = 0.0;
            _policy_armed_flag = false;
            _flip_flag = false;

            ros::param::get("/CEILING_HEIGHT",h_ceiling);
            ros::param::get("/LANDING_SLOWDOWN",landing_slowdown_flag);
        }

        // DEFINE FUNCTION PROTOTYPES
        void Load();
        void recvThread_RL();
        void controlThread();
        void global_stateCallback(const nav_msgs::Odometry::ConstPtr &msg);
        void OFCallback(const nav_msgs::Odometry::ConstPtr &msg);
        void imuCallback(const sensor_msgs::Imu::ConstPtr &msg);
        void RLCmd_Callback(const crazyflie_rl::RLCmd::ConstPtr &msg);
        void adjustSimSpeed(float speed_mult);

    private:
        // DEFINE PUBLISHERS AND SUBSCRIBERS
        ros::Publisher ctrl_Publisher;
        ros::Subscriber globalState_Subscriber;
        ros::Subscriber OF_Subscriber;
        ros::Subscriber RLCmd_Subscriber;
        ros::Subscriber imu_Subscriber;
        

        ros::ServiceClient client;

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

        // LOCAL CONTROLLER VARIABLES
        Eigen::Vector3d Kp_P;  // Pos. Gain
        Eigen::Vector3d Kd_P;  // Pos. Derivative Gain
        Eigen::Vector3d Ki_P;  // Pos. Integral Gain

        Eigen::Vector3d Kp_R;  // Rot. Gain
        Eigen::Vector3d Kd_R;  // Rot. Derivative Gain
        Eigen::Vector3d Ki_R;  // Rot. Integral Gain


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
        double _kd_xf = 1; // Pos. Derivative Gain Flag
        double _ki_xf = 1; // Pos. Integral Flag
        double _kp_Rf = 1; // Rot. Gain Flag
        double _kd_Rf = 1; // Rot. Derivative Gain Flag
        double _ki_Rf = 1; // Rot. Integral Flag

        bool _motorstop_flag = false;
        bool _Moment_flag = false;
        bool _tumbled = false;
        bool _tumble_detection = true;


        // XY POSITION PID
        float P_kp_xy = 0.5f;
        float P_kd_xy = 0.4f;
        float P_ki_xy = 0.1f*0;
        float i_range_xy = 0.3f;

        // Z POSITION PID
        float P_kp_z = 1.2f;
        float P_kd_z = 0.35f;
        float P_ki_z = 0.1f*0;
        float i_range_z = 0.25f;

        // XY ATTITUDE PID
        float R_kp_xy = 0.004f;
        float R_kd_xy = 0.0017f;
        float R_ki_xy = 0.0f;
        float i_range_R_xy = 1.0f;

        // Z ATTITUDE PID
        float R_kp_z = 0.003f;
        float R_kd_z = 0.001f;
        float R_ki_z = 0.002*0;
        float i_range_R_z = 0.5f;


        // MISC VARIABLES
        double h_ceiling;
        bool landing_slowdown_flag;
        



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

// Converts thrust in Newtons to their respective PWM values
static inline int32_t thrust2PWM(float f) 
{
    // Conversion values calculated from self motor analysis
    float a = 3.31e4;
    float b = 1.12e1;
    float c = 8.72;
    float d = 3.26e4;

    float s = 1.0f; // sign of value
    int32_t f_pwm = 0;

    s = f/fabsf(f);
    f = fabsf(f);
    
    f_pwm = a*tanf((f-c)/b)+d;

    return s*f_pwm;

}      

// Converts thrust in PWM to their respective Newton values
static inline float PWM2thrust(int32_t M_PWM) 
{
    // Conversion values calculated from PWM to Thrust Curve
    // Linear Fit: Thrust [g] = a*PWM + b
    float a = 3.31e4;
    float b = 1.12e1;
    float c = 8.72;
    float d = 3.26e4;

    float f = b*atan2f(M_PWM-d,a)+c;
    // float f = (a*M_PWM + b); // Convert thrust to grams

    if(f<0)
    {
      f = 0;
    }

    return f;
}


// Limit PWM value to accurate portion of motor curve (0 - 60,000)
uint16_t limitPWM(int32_t value)
{
  if(value > PWM_MAX)
  {
    value = PWM_MAX;
  }
  else if(value < 0)
  {
    value = 0;
  }

  return (uint16_t)value;
}