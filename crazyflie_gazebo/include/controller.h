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
            ctrl_Publisher = nh->advertise<crazyflie_msgs::CtrlData>("/ctrl_data",1);

            // NOTE: tcpNoDelay() removes delay where system is waiting for datapackets to be fully filled before sending;
            // instead of sending data as soon as it is available to match publishing rate (This is an issue with large messages like Odom or Custom)
            // Queue lengths are set to '1' so only the newest data is used


            // SENSORS
            globalState_Subscriber = nh->subscribe("/vicon_state",1,&Controller::global_stateCallback,this,ros::TransportHints().tcpNoDelay());
            OF_Subscriber = nh->subscribe("/OF_sensor",1,&Controller::OFCallback,this,ros::TransportHints().tcpNoDelay()); 
                

            // COMMANDS AND INFO

            // INIT VARIABLES TO DEFAULT VALUES (PREVENTS RANDOM VALUES FROM MEMORY)

            _RREV = 0.0;
            _OF_x = 0.0;
            _OF_y = 0.0;


            ros::param::get("/CEILING_HEIGHT",_H_CEILING);
            ros::param::get("/LANDING_SLOWDOWN",_LANDING_SLOWDOWN_FLAG);
            ros::param::get("/K_EP_SLOWDOWN",_K_EP_SLOWDOWN);
            ros::param::get("/SIM_SPEED",_SIM_SPEED);
            ros::param::get("/SIM_SLOWDOWN_SPEED",_SIM_SLOWDOWN_SPEED);
            ros::param::get("/CF_MASS",_CF_MASS);
            ros::param::get("/CTRL_OUTPUT_SLOWDOWN", _CTRL_OUTPUT_SLOWDOWN);


            m = _CF_MASS;
            h_ceiling = _H_CEILING;
        }

        // DEFINE FUNCTION PROTOTYPES
        void Load();
        void controllerGTC();
        void controllerGTCReset();
        void global_stateCallback(const nav_msgs::Odometry::ConstPtr &msg);
        void OFCallback(const nav_msgs::Odometry::ConstPtr &msg);



    private:
        // DEFINE PUBLISHERS AND SUBSCRIBERS
        ros::Publisher ctrl_Publisher;
        ros::Subscriber OF_Subscriber;


        // SENSORS
        ros::Subscriber globalState_Subscriber;

        // INITIALIZE ROS MSG VARIABLES
        geometry_msgs::Point _position; 
        geometry_msgs::Vector3 _velocity;
        geometry_msgs::Quaternion _quaternion;
        geometry_msgs::Vector3 _omega;


        // DEFINE THREAD OBJECTS
        std::thread controllerThread;
        std::thread senderThread_gazebo;

        bool _isRunning;



        // MISC VARIABLES AND CONSTANTS
        int _k_ep;
        float _H_CEILING;
        bool _LANDING_SLOWDOWN_FLAG;
        int _K_EP_SLOWDOWN;
        float _SIM_SPEED; 
        float _SIM_SLOWDOWN_SPEED;
        float _CF_MASS;
        int _CTRL_OUTPUT_SLOWDOWN;
        
        bool _TEST_FLAG = false;


        double _RREV = 0.0;  // [rad/s]
        double _OF_x = 0.0;  // [rad/s]
        double _OF_y = 0.0;  // [rad/s]

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

        float dt = (1.0f/500.0f);

        // CONTROLLER PARAMETERS
        bool attCtrlEnable = false;
        bool tumbled = false;
        bool motorstop_flag = false;
        bool errorReset = false;
        bool flip_flag = false;


        // INIT CTRL GAIN VECTORS
        struct vec Kp_p;    // Pos. Proportional Gains
        struct vec Kd_p;    // Pos. Derivative Gains
        struct vec Ki_p;    // Pos. Integral Gains  
        struct vec Kp_R;    // Rot. Proportional Gains
        struct vec Kd_R;    // Rot. Derivative Gains
        struct vec Ki_R;    // Rot. Integral Gains

        // STATE VALUES
        double _t;
        struct vec statePos = {0.0f,0.0f,0.0f};         // Pos [m]
        struct vec stateVel = {0.0f,0.0f,0.0f};         // Vel [m/s]
        struct quat stateQuat = {0.0f,0.0f,0.0f,1.0f};  // Orientation
        struct vec stateOmega = {0.0f,0.0f,0.0f};       // Angular Rate [rad/s]

        struct mat33 R; // Orientation as rotation matrix
        struct vec stateEul = {0.0f,0.0f,0.0f}; // Pose in Euler Angles [YZX Notation]

        // OPTICAL FLOW STATES
        float RREV = 0.0f;  // [rad/s]
        float OF_x = 0.0f;  // [rad/s]
        float OF_y = 0.0f;  // [rad/s] 

        


        // DESIRED STATES
        struct vec x_d = {0.0f,0.0f,0.0f};  // Pos-desired [m]
        struct vec v_d = {0.0f,0.0f,0.0f};  // Vel-desired [m/s]
        struct vec a_d = {0.0f,0.0f,0.0f};  // Acc-desired [m/s^2]

        struct quat quat_d = {0.0f,0.0f,0.0f,1.0f}; // Orientation-desired [qx,qy,qz,qw]
        struct vec eul_d = {0.0f,0.0f,0.0f};        // Euler Angle-desired [rad? deg? TBD]
        struct vec omega_d = {0.0f,0.0f,0.0f};      // Omega-desired [rad/s]
        struct vec domega_d = {0.0f,0.0f,0.0f};     // Ang. Acc-desired [rad/s^2]

        struct vec b1_d = {1.0f,0.0f,0.0f};    // Desired body x-axis in global coord. [x,y,z]
        struct vec b2_d;    // Desired body y-axis in global coord.
        struct vec b3_d;    // Desired body z-axis in global coord.
        struct vec b3;      // Current body z-axis in global coord.

        struct mat33 R_d;   // Desired rotational matrix from b_d vectors
        struct vec e_3 = {0.0f, 0.0f, 1.0f}; // Global z-axis

        // STATE ERRORS
        struct vec e_x;     // Pos-error [m]
        struct vec e_v;     // Vel-error [m/s]
        struct vec e_PI;    // Pos. Integral-error [m*s]

        struct vec e_R;     // Rotation-error [rad]
        struct vec e_w;     // Omega-error [rad/s]
        struct vec e_RI;    // Rot. Integral-error [rad*s]

        struct vec F_thrust_ideal;           // Ideal thrust vector
        float F_thrust = 0.0f;               // Desired body thrust [N]
        float F_thrust_max = 0.64f;          // Max possible body thrust [N}]
        struct vec M;                        // Desired body moments [Nm]
        struct vec M_d = {0.0f,0.0f,0.0f};   // Desired moment [N*mm]
        float Moment_flag = false;


        // MOTOR THRUSTS
        float f_thrust; // Motor thrust - Thrust [N]
        float f_roll;   // Motor thrust - Roll   [N]
        float f_pitch;  // Motor thrust - Pitch  [N]
        float f_yaw;    // Motor thrust - Yaw    [N]


        float f_thrust_g = 0.0; // Motor thrust - Thrust [g]
        float f_roll_g = 0.0;   // Motor thrust - Roll   [g]
        float f_pitch_g = 0.0;  // Motor thrust - Pitch  [g]
        float f_yaw_g = 0.0;    // Motor thrust - Yaw    [g]



        // MOTOR VARIABLES
        uint32_t M1_pwm = 0; 
        uint32_t M2_pwm = 0; 
        uint32_t M3_pwm = 0; 
        uint32_t M4_pwm = 0; 

        // MOTOR SPEEDS
        float MS1 = 0.0;
        float MS2 = 0.0;
        float MS3 = 0.0;
        float MS4 = 0.0;

        float motorspeed[4]; // Motorspeed sent to plugin

        // TEMPORARY CALC VECS/MATRICES
        struct vec temp1_v; 
        struct vec temp2_v;
        struct vec temp3_v;
        struct vec temp4_v;
        struct mat33 temp1_m;  

        struct vec P_effort;
        struct vec R_effort;

        struct mat33 RdT_R; // Rd' * R
        struct mat33 RT_Rd; // R' * Rd
        struct vec Gyro_dyn;
        
                

        

        

        

        


        


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
    controllerThread = std::thread(&Controller::controllerGTC, this);


}


void Controller::global_stateCallback(const nav_msgs::Odometry::ConstPtr &msg){


    // Follow msg names from message details - "rqt -s rqt_msg" 
    
    
    // SET STATE VALUES INTO CLASS STATE VARIABLES
    _t = msg->header.stamp.toSec();
    
    _position = msg->pose.pose.position; 
    _velocity = msg->twist.twist.linear;
    _quaternion = msg->pose.pose.orientation;
    _omega = msg->twist.twist.angular;

    
}


void Controller::OFCallback(const nav_msgs::Odometry::ConstPtr &msg){

    const geometry_msgs::Point position = msg->pose.pose.position; 
    const geometry_msgs::Vector3 velocity = msg->twist.twist.linear;

    
    double d = _H_CEILING-position.z; // h_ceiling - height

    // SET SENSOR VALUES INTO CLASS VARIABLES
    // _RREV = msg->RREV;
    // _OF_x = msg->OF_x;
    // _OF_y = msg->OF_y;

    _RREV = velocity.z/d;
    _OF_x = -velocity.y/d;
    _OF_y = -velocity.x/d;
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

static inline void printvec(struct vec v){

    std::cout << v.x << "\t" << v.y << "\t" << v.z << std::endl;
    
}

