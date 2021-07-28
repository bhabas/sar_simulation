#include <iostream>
#include <thread>
#include <Eigen/Dense>
#include <cmath>        // std::abs
#include <math.h>       


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

using namespace Eigen;
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
            globalState_Subscriber = nh->subscribe("/global_state",1,&Controller::global_stateCallback,this,ros::TransportHints().tcpNoDelay());
            OF_Subscriber = nh->subscribe("/OF_sensor",1,&Controller::OFCallback,this,ros::TransportHints().tcpNoDelay()); 
            imu_Subscriber = nh->subscribe("/imu",1,&Controller::imuCallback,this);
            ceilingFT_Subcriber = nh->subscribe("/ceiling_force_sensor",5,&Controller::ceilingFTCallback,this,ros::TransportHints().tcpNoDelay());
            padContact_Subcriber = nh->subscribe("/pad_connections",5,&Controller::pad_connectCallback,this,ros::TransportHints().tcpNoDelay());

            // COMMANDS AND INFO
            RLData_Subscriber = nh->subscribe("/rl_data",5,&Controller::RLData_Callback,this);
            RLCmd_Subscriber = nh->subscribe("/rl_ctrl",50,&Controller::RLCmd_Callback,this);

            SimSpeed_Client = nh->serviceClient<gazebo_msgs::SetPhysicsProperties>("/gazebo/set_physics_properties");



            
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
            _impact_flag = false;
            _slowdown_type = 0;
            ros::param::get("/CEILING_HEIGHT",_H_CEILING);
            ros::param::get("/LANDING_SLOWDOWN",_LANDING_SLOWDOWN_FLAG);
            ros::param::get("/K_EP_SLOWDOWN",_K_EP_SLOWDOWN);
            ros::param::get("/SIM_SPEED",_SIM_SPEED);
            ros::param::get("/SIM_SLOWDOWN_SPEED",_SIM_SLOWDOWN_SPEED);
            ros::param::get("/CF_MASS",_CF_MASS);
        }

        // DEFINE FUNCTION PROTOTYPES
        void Load();
        void recvThread_RL();
        void controlThread();
        void global_stateCallback(const nav_msgs::Odometry::ConstPtr &msg);
        void OFCallback(const nav_msgs::Odometry::ConstPtr &msg);
        void imuCallback(const sensor_msgs::Imu::ConstPtr &msg);
        void pad_connectCallback(const crazyflie_msgs::PadConnect::ConstPtr &msg);
        void RLCmd_Callback(const crazyflie_msgs::RLCmd::ConstPtr &msg);
        void RLData_Callback(const crazyflie_msgs::RLData::ConstPtr &msg);
        void ceilingFTCallback(const crazyflie_msgs::ImpactData::ConstPtr &msg);
        void adjustSimSpeed(float speed_mult);

    private:
        // DEFINE PUBLISHERS AND SUBSCRIBERS
        ros::Publisher ctrl_Publisher;

        // SENSORS
        ros::Subscriber globalState_Subscriber;
        ros::Subscriber OF_Subscriber;
        ros::Subscriber imu_Subscriber;
        ros::Subscriber ceilingFT_Subcriber;
        ros::Subscriber padContact_Subcriber;

        // COMMANDS AND INFO
        ros::Subscriber RLCmd_Subscriber;
        ros::Subscriber RLData_Subscriber;

        ros::ServiceClient SimSpeed_Client;

        // DEFINE THREAD OBJECTS
        std::thread controllerThread;
        std::thread senderThread_gazebo;

        

        // DEFINE CLASS VARIABLES (Similar to Python's class variables)
        // Leading '_' represents a class variable that works across functions
        bool _isRunning;
        Eigen::Matrix<double,5,1> _ctrl_cmd;


        // CONTROLLER GAIN VALUES
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
       

        // STATE VALUES
        double _t;
        Eigen::Vector3d _pos;   // Current position [m]
        Eigen::Vector3d _vel;   // Current velocity [m]
        Eigen::Vector4d _quat;  // Current attitude // [qw,qx,qy,qz]
        Eigen::Vector3d _omega; // Current angular velocity [rad/s]

        double _RREV;
        double _OF_x;
        double _OF_y; 

        

        // SETPOINTS
        Eigen::Vector3d _x_d;       // Pos-desired [m]  
        Eigen::Vector3d _v_d;       // Velocity-desired [m/s]
        Eigen::Vector3d _a_d;       // Acceleration-desired [m/s]
        Eigen::Vector3d _b1_d;      // Desired body-fixed x-axis in terms of global axes
        Eigen::Vector3d _omega_d;   // Omega-desired [rad/s]
        Eigen::Vector3d _M_d;       // Moment-desired [N*m]

        Eigen::Vector3d _eul_d;     // Euler-desired [rad?]
        bool _eul_flag = false;     // Flag to enable attitude control
        Eigen::Matrix3f _R_d_custom;// Rotation-desired (YZX Euler notation)

        

        // LOGGED FLIP VALUES
        Eigen::Vector3d _pos_flip;    // Flip trigger position [m]
        Eigen::Vector3d _vel_flip;    // Flip trigger velocity [m]
        Eigen::Vector4d _quat_flip;   // Flip trigger attitude // [qw,qx,qy,qz]
        Eigen::Vector3d _omega_flip;  // Flip trigger angular velocity [rad/s]

        float _OF_y_flip = 0.0;
        float _OF_x_flip = 0.0;
        float _RREV_flip = 0.0;

        float _f_thrust_g_flip = 0.0;
        float _f_roll_g_flip = 0.0;
        float _f_pitch_g_flip = 0.0;
        float _f_yaw_g_flip = 0.0;


        

        // POLICY FLAGS AND VALUES
        double _RREV_thr;
        double _G1;
        double _G2;
        bool _policy_armed_flag;
        bool _flip_flag;
        bool _impact_flag;
        int _slowdown_type;
        

        // CONTROLLER FLAGS
        bool _motorstop_flag = false;
        bool _Moment_flag = false;
        bool _tumbled = false;
        bool _tumble_detection = true;

        // CONTROLLER GAIN FLAGS
        double _kp_xf = 1; // Pos. Gain Flag
        double _kd_xf = 1; // Pos. Derivative Gain Flag
        double _ki_xf = 1; // Pos. Integral Flag
        double _kp_Rf = 1; // Rot. Gain Flag
        double _kd_Rf = 1; // Rot. Derivative Gain Flag
        double _ki_Rf = 1; // Rot. Integral Flag

      
        // CONTROLLER GAIN VECTORS
        Eigen::Vector3d Kp_P;  // Pos. Gain
        Eigen::Vector3d Kd_P;  // Pos. Derivative Gain
        Eigen::Vector3d Ki_P;  // Pos. Integral Gain

        Eigen::Vector3d Kp_R;  // Rot. Gain
        Eigen::Vector3d Kd_R;  // Rot. Derivative Gain
        Eigen::Vector3d Ki_R;  // Rot. Integral Gain

        


        // MISC VARIABLES AND CONSTANTS
        int _k_ep;
        double _H_CEILING;
        bool _LANDING_SLOWDOWN_FLAG;
        int _K_EP_SLOWDOWN;
        float _SIM_SPEED; 
        float _SIM_SLOWDOWN_SPEED;
        float _CF_MASS;
        
        bool _TEST_FLAG = false;


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

// CUSTOM EIGEN FUNCTIONS FOR HAT AND DEHAT OPERATORS
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

    SimSpeed_Client.call(srv);
}

// UNUSED CALLBACK
void Controller::imuCallback(const sensor_msgs::Imu::ConstPtr &msg){
    int a = 0;
}

// UNSUSED CALLBACK
void Controller::ceilingFTCallback(const crazyflie_msgs::ImpactData::ConstPtr &msg)
{
    _impact_flag = msg->impact_flag;
}

// TRIGGER IMPACT FLAG WHENEVER PAD CONNECTION MSG RECEIVED
void Controller::pad_connectCallback(const crazyflie_msgs::PadConnect::ConstPtr &msg){
    // _impact_flag = true;
}

void Controller::global_stateCallback(const nav_msgs::Odometry::ConstPtr &msg){

    // SIMPLIFY STATE VALUES FROM TOPIC
    // Follow msg names from message details - "rqt -s rqt_msg" 
    
    const geometry_msgs::Point position = msg->pose.pose.position; 
    const geometry_msgs::Vector3 velocity = msg->twist.twist.linear;


    // SET STATE VALUES INTO CLASS STATE VARIABLES
    _t = msg->header.stamp.toSec();
    _pos << position.x, position.y, position.z;
    _vel << velocity.x, velocity.y, velocity.z;

    // This stuff should come from IMU callback but lockstep broke that topic for some reason
    const geometry_msgs::Quaternion quaternion = msg->pose.pose.orientation;
    const geometry_msgs::Vector3 omega = msg->twist.twist.angular;

    _quat << quaternion.w, quaternion.x, quaternion.y, quaternion.z, 
    _omega << omega.x, omega.y, omega.z;

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