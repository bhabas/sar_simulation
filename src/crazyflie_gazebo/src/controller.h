#include <iostream>
#include <thread>
#include <Eigen/Dense>

// ROS Includes
#include <ros/ros.h>
#include "crazyflie_gazebo/CtrlData.h"
#include "gazebo_communication_pkg/GlobalState.h"
#include "crazyflie_rl/RLCmd.h"


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
            ctrl_Publisher = nh->advertise<crazyflie_gazebo::CtrlData>("/ctrl_data",10);

            globalState_Subscriber = nh->subscribe("/global_state",1000,&Controller::global_stateCallback,this);
            RLCmd_Subscriber = nh->subscribe("/rl_ctrl",10,&Controller::RLCmd_Callback,this);

            ctrl_cmd << 404,0,0,0,0;

            
            // INIT VARIABLES TO DEFAULT VALUES (PREVENTS RANDOM VALUES FROM MEMORY)
            _t = 0.0; 
            _pos << 0,0,0;
            _vel << 0,0,0;
            _quat << 1,0,0,0;
            _omega << 0,0,0;



            _x_d << 0,0,0.3;
            _v_d << 0,0,0;
            _a_d << 0,0,0;
            _b1_d << 1,0,0;
            _omega_d << 0,0,0;

            _kp_x << 0.1,0.1,0.11;
            _kd_x << 0.08,0.08,0.08;
            _kp_R << 0.05,0.05,0.05;
            _kd_R << 0.005,0.005,0.005;

            _kp_omega << 0.05,0.05,0.0;



        }

        // DEFINE FUNCTION PROTOTYPES
        void Load();
        void recvThread_RL();
        void controlThread();
        void global_stateCallback(const gazebo_communication_pkg::GlobalState::ConstPtr &msg);
        void RLCmd_Callback(const crazyflie_rl::RLCmd::ConstPtr &msg);

    private:
        // DEFINE PUBLISHERS AND SUBSCRIBERS
        ros::Publisher ctrl_Publisher;
        ros::Subscriber globalState_Subscriber;
        ros::Subscriber RLCmd_Subscriber;

        // DEFINE THREAD OBJECTS
        std::thread controllerThread;
        std::thread senderThread_gazebo;

        

        // DEFINE CLASS VARIABLES (Similar to Python's class variables)
        // Leading '_' represents a class variable that works across functions
        bool _isRunning;
        double control_cmd_recvd[5];
        Eigen::Matrix<double,5,1> ctrl_cmd;
       

        float _t;
        Eigen::Vector3d _pos;   // Current position [m]
        Eigen::Vector3d _vel;   // Current velocity [m]
        Eigen::Vector4d _quat;  // Current attitude [rad] (quat form)
        Eigen::Vector3d _omega; // Current angular velocity [rad/s]


        Eigen::Vector3d _x_d;       // Pos-desired [m]  
        Eigen::Vector3d _v_d;       // Velocity-desired [m/s]
        Eigen::Vector3d _a_d;       // Acceleration-desired [m/s]
        Eigen::Vector3d _b1_d;      // Desired body-fixed x-axis in terms of global axes
        Eigen::Vector3d _omega_d;   // Omega-desired [rad/s]

        Eigen::Vector3d _kp_x; // Pos. Gain
        Eigen::Vector3d _kd_x; // Pos. derivative Gain
        Eigen::Vector3d _kp_R; // Rot. Gain
        Eigen::Vector3d _kd_R; // Rot. derivative Gain

        
        Eigen::Vector3d _kp_omega; // Flip proportional Gain
        // Omega proportional gain (similar to kd_R but that's for damping and this is to achieve omega_d)
        // kd_R is great for stabilization but for flip manuevers it's too sensitive and 
        // saturates the motors causing instability during the rotation

        

        // CONTROLLER FLAGS
        double _kp_xf = 1; // Pos. Gain Flag
        double _kd_xf = 1; // Pos. derivative Gain Flag
        double _kp_Rf = 1; // Rot. Gain Flag
        double _kd_Rf = 1; // Rot. derivative Gain Flag

        double _motorstop_flag = 0;
        double _flip_flag = 1;



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

        // DEFINE RL ADDRESS VARIABLES
        int RL_PORT;
        struct sockaddr_in addr_RL;
        socklen_t addr_RL_len;

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