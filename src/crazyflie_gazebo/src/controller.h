#include <iostream>
#include <thread>
#include <Eigen/Dense>

// ROS Includes
#include <ros/ros.h>
#include "crazyflie_gazebo/CtrlData.h"
#include <gazebo_communication_pkg/GlobalState.h>

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
            globalState_Subscriber = nh->subscribe("/global_state",1000,
            &Controller::callback_number,this);


            _pos << 0.0,0.0,0.0;
            _vel << 0.0,0.0,0.0;
            _quat << 0.0,0.0,0.0,0.0;
            _omega << 0.0,0.0,0.0;
        }

        // DEFINE FUNCTION PROTOTYPES
        void Load();
        void recvThread_RL();
        void controlThread();
        void callback_number(const gazebo_communication_pkg::GlobalState::ConstPtr &msg);

    private:
        // DEFINE PUBLISHERS AND SUBSCRIBERS
        ros::Publisher ctrl_Publisher;
        ros::Subscriber globalState_Subscriber;
        ros::Subscriber RL_Subscriber;

        // DEFINE THREAD OBJECTS
        std::thread controllerThread;
        std::thread senderThread_gazebo;
        std::thread receiverThread_RL;
        

        // DEFINE CLASS VARIABLES (Similar to Python's class variables)
        // Leading '_' represents a class variable that works across functions
        bool _isRunning;
        double control_cmd_recvd[5];
       

        float _t;
        Eigen::Vector3d _pos;
        Eigen::Vector3d _vel;
        Eigen::Vector4d _quat;
        Eigen::Vector3d _omega;



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