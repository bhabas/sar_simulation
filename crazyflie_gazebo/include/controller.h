#include <iostream>
#include <thread>
#include <Eigen/Dense>
#include <cmath>        // std::abs
#include <math.h>       
#include "math3d.h"


// ROS Includes
#include <ros/ros.h>
#include "crazyflie_msgs/CtrlData.h"
#include "crazyflie_msgs/ImpactData.h"
#include "crazyflie_msgs/RLCmd.h"
#include "crazyflie_msgs/RLData.h"
#include "crazyflie_msgs/PadConnect.h"


#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "gazebo_msgs/SetPhysicsProperties.h"

// Socket Includes
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>

#include "readerwriterqueue.h"


using namespace std;

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

            // NOTE: tcpNoDelay() removes delay where system is waiting for datapackets to be fully filled before sending;
            // instead of sending data as soon as it is available to match publishing rate (This is an issue with large messages like Odom or Custom)
            // Queue lengths are set to '1' so only the newest data is used
            // SENSORS
            globalState_Subscriber = nh->subscribe("/vicon_state",1,&Controller::global_stateCallback,this,ros::TransportHints().tcpNoDelay());
                

        }

        // DEFINE FUNCTION PROTOTYPES
        void Load();
        void controlThread();
        void global_stateCallback(const nav_msgs::Odometry::ConstPtr &msg);


    private:

        // SENSORS
        ros::Subscriber globalState_Subscriber;

        // DEFINE THREAD OBJECTS
        std::thread controllerThread;
        std::thread senderThread_gazebo;

        bool _isRunning;

        // SYSTEM PARAMETERS
        float m = 0.037f; // [g]
        float g = 9.81f;
        struct mat33 J; // Rotational Inertia Matrix [kg*m^2]
        float h_ceiling = 2.10f; // [m]

        float d = 0.040f;    // COM to Prop [m]
        float dp = 0.028284; // COM to Prop along x-axis [m]
                                    // [dp = d*sin(45 deg)]

        float const kf = 2.2e-8f;    // Thrust Coeff [N/(rad/s)^2]
        float const c_tf = 0.00618f;  // Moment Coeff [Nm/N]
        

        // INIT CTRL GAIN VECTORS
        struct vec Kp_p; // Pos. Proportional Gains
        struct vec Kd_p; // Pos. Derivative Gains
        struct vec Ki_p; // Pos. Integral Gains  
        struct vec Kp_R; // Rot. Proportional Gains
        struct vec Kd_R; // Rot. Derivative Gains
        struct vec Ki_R; // Rot. Integral Gains

        // STATE VALUES
        struct vec statePos = {0.0f,0.0f,0.0f};         // Pos [m]
        struct vec stateVel = {0.0f,0.0f,0.0f};         // Vel [m/s]
        struct quat stateQuat = {0.0f,0.0f,0.0f,1.0f};  // Orientation
        struct vec stateOmega = {0.0f,0.0f,0.0f};       // Angular Rate [rad/s]

        struct mat33 R; // Orientation as rotation matrix
        struct vec stateEul = {0.0f,0.0f,0.0f}; // Pose in Euler Angles [YZX Notation]


        // OPTICAL FLOW STATES
        float RREV = 0.0f; // [rad/s]
        float OF_x = 0.0f; // [rad/s]
        float OF_y = 0.0f; // [rad/s] 
        bool flip_flag = false;


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


void Controller::Load()
{
    cout << setprecision(3);
    cout << fixed;
    _isRunning = true;

    // INIT FIRST CONTROLLER SOCKET (COMMUNICATES W/ MAVLINK PORT:18080)
    Ctrl_Mavlink_socket = socket(AF_INET, SOCK_DGRAM, 0); // DGRAM is for UDP communication (Send data but don't care if it's recieved)
    Ctrl_Mavlink_socket_SNDBUF = 16;    // 4 floats [16 bytes] for Motorspeeds       
    Ctrl_Mavlink_socket_RCVBUF = 144;   // 18 doubles [144 bytes] for State array
    Ctrl_Mavlink_socket_PORT = 18070;   // Port for this socket

    // SET EXPECTED BUFFER SIZES
    if (setsockopt(Ctrl_Mavlink_socket, SOL_SOCKET, SO_SNDBUF, &Ctrl_Mavlink_socket_SNDBUF, sizeof(Ctrl_Mavlink_socket_SNDBUF))<0)
        cout<<"[FAILED] Ctrl_Mavlink_socket: Setting SNDBUF"<<endl;
    if (setsockopt(Ctrl_Mavlink_socket, SOL_SOCKET, SO_RCVBUF, &Ctrl_Mavlink_socket_RCVBUF, sizeof(Ctrl_Mavlink_socket_RCVBUF))<0)
        cout<<"[FAILED] Ctrl_Mavlink_socket: Setting RCVBUF"<<endl;
    int enable = 1; // Fix for error if socket hasn't close correctly when restarting program
    if (setsockopt(Ctrl_Mavlink_socket, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(int)) < 0)
        cout <<"help me"<< endl;

    // SET SOCKET SETTINGS
    memset(&addr_Ctrl_Mavlink, 0, sizeof(addr_Ctrl_Mavlink));
    addr_Ctrl_Mavlink.sin_family = AF_INET;
    addr_Ctrl_Mavlink.sin_addr.s_addr = htonl(INADDR_ANY);//inet_addr("0.0.0.0");
    addr_Ctrl_Mavlink.sin_port = htons(Ctrl_Mavlink_socket_PORT);

    // BIND ADDRESS TO CONTROLLER SOCKET (PORT:18070)
    if (bind(Ctrl_Mavlink_socket, (struct sockaddr*)&addr_Ctrl_Mavlink, sizeof(addr_Ctrl_Mavlink)) < 0)
        cout<<"[FAILED] Ctrl_Mavlink_socket: Binding address to socket"<<endl;
    else
        cout<<"[SUCCESS] Ctrl_Mavlink_socket: Binding address to socket"<<endl; 


    



    // INIT ADDRESS FOR MAVLINK SOCKET (PORT: 18080)
    Mavlink_PORT = 18080;
    memset(&addr_Mavlink, 0, sizeof(addr_Mavlink));
    addr_Mavlink.sin_family = AF_INET;
    addr_Mavlink.sin_addr.s_addr = htonl(INADDR_ANY);
    addr_Mavlink.sin_port = htons(Mavlink_PORT);
    addr_Mavlink_len = sizeof(addr_Mavlink);



    // MOTORSPEED TO MAVLINK TEST (VISUALLY CONFIRMS THINGS ARE WORKING IN SIM)
    float msg[4] = {1900,1900,1900,1900};
    int msg_len = 0;
    for(int k=0; k<2; k++)
        // To Gazebo socket, send msg of len(msg)
        msg_len = sendto(Ctrl_Mavlink_socket, msg, sizeof(msg),0, (struct sockaddr*)&addr_Mavlink, sizeof(addr_Mavlink));
    if(msg_len<0)
        cout<<"[FAILED] Ctrl_Mavlink_socket: Sending test motor speeds to Mavlink. Threads will mutual lock!"<<endl; // Not sure what mutual lock means
    else
        cout<<"[SUCCESS] Ctrl_Mavlink_socket: Sending test motor speeds to Mavlink. Avoiding mutual locking between threads!"<<endl;

    // START COMMUNICATION THREADS
    controllerThread = std::thread(&Controller::controlThread, this);


}


void Controller::global_stateCallback(const nav_msgs::Odometry::ConstPtr &msg){



}