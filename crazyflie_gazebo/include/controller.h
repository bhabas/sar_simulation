
// C++ LIBRARIES
#include <iostream>
#include <thread>
#include <cmath>        // std::abs

// ROS Includes
#include <ros/ros.h>
#include "crazyflie_msgs/CtrlData.h"
#include "crazyflie_msgs/ImpactData.h"
#include "crazyflie_msgs/RLCmd.h"
#include "crazyflie_msgs/RLData.h"
#include "crazyflie_msgs/PadConnect.h"
#include "crazyflie_msgs/Policy_Values.h"
#include "crazyflie_msgs/MS.h"


#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "gazebo_msgs/SetPhysicsProperties.h"


// STANDARD LIBRARIES
#include <math.h>       
#include <stdio.h>
#include <stdint.h>

// CF LIBRARIES
#include "math3d.h"
#include "stabilizer_types.h"
#include "nml.h"

using namespace std;


#define PWM_MAX 60000
#define f_MAX (16.5)
#define g2Newton (9.81f/1000.0f)
#define Newton2g (1000.0f/9.81f)

class Controller
{
    public:
        // CONSTRUCTOR TO START PUBLISHERS AND SUBSCRIBERS (Similar to Python's __init__() )
        Controller(ros::NodeHandle *nh){
            
            

            // ENVIRONMENT SENSORS
            viconState_Subscriber = nh->subscribe("/env/vicon_state",1,&Controller::vicon_Callback,this,ros::TransportHints().tcpNoDelay());
            
            // BODY SENSORS
            imu_Subscriber = nh->subscribe("/cf1/imu",1,&Controller::imu_Callback,this,ros::TransportHints().tcpNoDelay());
            OF_Subscriber = nh->subscribe("/cf1/OF_sensor",1,&Controller::OF_Callback,this,ros::TransportHints().tcpNoDelay()); 

            // COMMANDS AND INFO
            CMD_Subscriber = nh->subscribe("/rl_ctrl",50,&Controller::CMD_Callback,this);
            ctrl_Publisher = nh->advertise<crazyflie_msgs::CtrlData>("/ctrl_data",1);
            MS_Publisher = nh->advertise<crazyflie_msgs::MS>("/MS",1);




            state.position.x = 0.0f;
            state.position.y = 0.0f;
            state.position.z = 0.0f;

            state.velocity.x = 0.0f;
            state.velocity.y = 0.0f;
            state.velocity.z = 0.0f;

            state.attitudeQuaternion.x = 0.0f;
            state.attitudeQuaternion.y = 0.0f;
            state.attitudeQuaternion.z = 0.0f;
            state.attitudeQuaternion.w = 1.0f;

            state.acc.x = 0.0f;
            state.acc.y = 0.0f;
            state.acc.z = 0.0f;

            sensorData.gyro.x = 0.0f;
            sensorData.gyro.y = 0.0f;
            sensorData.gyro.z = 0.0f;



            setpoint.position.x = 0.0f;
            setpoint.position.y = 0.0f;
            setpoint.position.z = 0.4f;

            setpoint.velocity.x = 0.0f;
            setpoint.velocity.y = 0.0f;
            setpoint.velocity.z = 0.0f;

            setpoint.acceleration.x = 0.0f;
            setpoint.acceleration.y = 0.0f;
            setpoint.acceleration.z = 0.0f;

            setpoint.cmd_type = 101;
            setpoint.cmd_val1 = 0.0f;
            setpoint.cmd_val2 = 0.0f;
            setpoint.cmd_val3 = 0.0f;
            setpoint.cmd_flag = 0.0f;


            // START CONTROLLER
            controllerThread = std::thread(&Controller::startController, this);           
        }

        // DEFINE FUNCTION PROTOTYPES
        void startController();
        void vicon_Callback(const nav_msgs::Odometry::ConstPtr &msg);
        void imu_Callback(const sensor_msgs::Imu::ConstPtr &msg);
        void OF_Callback(const nav_msgs::Odometry::ConstPtr &msg);   
        void CMD_Callback(const crazyflie_msgs::RLCmd::ConstPtr &msg);


        crazyflie_msgs::MS MS_msg;
        crazyflie_msgs::CtrlData ctrl_msg;

        ros::Publisher MS_Publisher;
        ros::Publisher ctrl_Publisher;

        float _t;




    private:

        // SENSORS
        ros::Subscriber viconState_Subscriber;
        ros::Subscriber imu_Subscriber;
        ros::Subscriber OF_Subscriber;
        ros::Subscriber CMD_Subscriber;

        // INITIALIZE ROS MSG VARIABLES
        geometry_msgs::Point _position; 
        geometry_msgs::Vector3 _velocity;
        geometry_msgs::Quaternion _quaternion;
        geometry_msgs::Vector3 _omega;
        geometry_msgs::Vector3 _accel;

        float _H_CEILING = 2.10f;


        float _RREV;
        float _OF_x;
        float _OF_y;



        // DEFINE THREAD OBJECTS
        std::thread controllerThread;

        control_t control;
        setpoint_t setpoint;
        sensorData_t sensorData;
        state_t state;
        uint32_t tick = 0;
        


};




// FUNCTION PRIMITIVES
void stateEstimator(state_t *state, sensorData_t *sensors, control_t *control, const uint32_t tick);
void commanderGetSetpoint(setpoint_t *setpoint, const state_t *state);
void controllerGTCInit(void);
bool controllerGTCTest(void);
void controllerGTCReset(void);
void controllerGTCTraj(void);
void controllerGTC(control_t *control, setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick,
                                         Controller* ctrl);
void GTC_Command(setpoint_t *setpoint);

// SYSTEM PARAMETERS
static float m = 0.0376; // [g]
static float g = 9.81f;
struct mat33 J; // Rotational Inertia Matrix [kg*m^2]
static float h_ceiling = 2.10f; // [m]

static float d = 0.040f;    // COM to Prop [m]
static float dp = 0.028284; // COM to Prop along x-axis [m]
                            // [dp = d*sin(45 deg)]

static float const kf = 2.2e-8f;    // Thrust Coeff [N/(rad/s)^2]
static float const c_tf = 0.00618f;  // Moment Coeff [Nm/N]


// LOGGING VARIABLES
static float cmd_thrust;
static float cmd_roll;
static float cmd_pitch;
static float cmd_yaw;

// STATE ERRORS
static struct vec e_x;  // Pos-error [m]
static struct vec e_v;  // Vel-error [m/s]
static struct vec e_PI;  // Pos. Integral-error [m*s]

static struct vec e_R;  // Rotation-error [rad]
static struct vec e_w;  // Omega-error [rad/s]
static struct vec e_RI; // Rot. Integral-error [rad*s]

// STATE VALUES
static struct vec statePos = {0.0f,0.0f,0.0f};         // Pos [m]
static struct vec stateVel = {0.0f,0.0f,0.0f};         // Vel [m/s]
static struct quat stateQuat = {0.0f,0.0f,0.0f,1.0f};  // Orientation
static struct vec stateOmega = {0.0f,0.0f,0.0f};       // Angular Rate [rad/s]

static struct mat33 R; // Orientation as rotation matrix
static struct vec stateEul = {0.0f,0.0f,0.0f}; // Pose in Euler Angles [YZX Notation]

// DESIRED STATES
static struct vec x_d = {0.0f,0.0f,0.0f}; // Pos-desired [m]
static struct vec v_d = {0.0f,0.0f,0.0f}; // Vel-desired [m/s]
static struct vec a_d = {0.0f,0.0f,0.0f}; // Acc-desired [m/s^2]

static struct quat quat_d = {0.0f,0.0f,0.0f,1.0f}; // Orientation-desired [qx,qy,qz,qw]
static struct vec eul_d = {0.0f,0.0f,0.0f};        // Euler Angle-desired [rad? deg? TBD]
static struct vec omega_d = {0.0f,0.0f,0.0f};      // Omega-desired [rad/s]
static struct vec domega_d = {0.0f,0.0f,0.0f};     // Ang. Acc-desired [rad/s^2]

static struct vec b1_d = {1.0f,0.0f,0.0f};    // Desired body x-axis in global coord. [x,y,z]
static struct vec b2_d;    // Desired body y-axis in global coord.
static struct vec b3_d;    // Desired body z-axis in global coord.
static struct vec b3;      // Current body z-axis in global coord.

static struct mat33 R_d;   // Desired rotational matrix from b_d vectors





static struct vec e_3 = {0.0f, 0.0f, 1.0f}; // Global z-axis

static struct vec F_thrust_ideal;           // Ideal thrust vector
static float F_thrust = 0.0f;               // Desired body thrust [N]
static float F_thrust_max = 0.64f;          // Max possible body thrust [N}]
static struct vec M;                        // Desired body moments [Nm]
static struct vec M_d = {0.0f,0.0f,0.0f};   // Desired moment [N*mm]
static float Moment_flag = false;


// TEMPORARY CALC VECS/MATRICES
static struct vec temp1_v; 
static struct vec temp2_v;
static struct vec temp3_v;
static struct vec temp4_v;
static struct mat33 temp1_m;  

static struct vec P_effort;
static struct vec R_effort;

static struct mat33 RdT_R; // Rd' * R
static struct mat33 RT_Rd; // R' * Rd
static struct vec Gyro_dyn;



// MOTOR THRUSTS
static float f_thrust_g; // Motor thrust - Thrust [N]
static float f_roll_g;   // Motor thrust - Roll   [N]
static float f_pitch_g;  // Motor thrust - Pitch  [N]
static float f_yaw_g;    // Motor thrust - Yaw    [N]

static int32_t f_thrust_pwm; 
static int32_t f_roll_pwm;   
static int32_t f_pitch_pwm; 
static int32_t f_yaw_pwm;  


// INIT CTRL GAIN VECTORS
static struct vec Kp_p; // Pos. Proportional Gains
static struct vec Kd_p; // Pos. Derivative Gains
static struct vec Ki_p; // Pos. Integral Gains  

static struct vec Kp_R; // Rot. Proportional Gains
static struct vec Kd_R; // Rot. Derivative Gains
static struct vec Ki_R; // Rot. Integral Gains

static float dt = (float)(1.0f/RATE_500_HZ);

// CONTROLLER PARAMETERS
static bool attCtrlEnable = false;
static bool tumbled = false;
static bool motorstop_flag = false;
static bool errorReset = false;
static bool tumble_detection = true;

// OPTICAL FLOW STATES
static float RREV = 0.0f; // [rad/s]
static float OF_x = 0.0f; // [rad/s]
static float OF_y = 0.0f; // [rad/s] 
static bool flip_flag = false;





// STATE VALUES AT FLIP TRIGGER
static float RREV_tr = 0.0f;
static float OF_x_tr = 0.0f;
static float OF_y_tr = 0.0f;

static struct vec statePos_tr = {0.0f,0.0f,0.0f};         // Pos [m]
static struct vec stateVel_tr = {0.0f,0.0f,0.0f};         // Vel [m/s]
static struct quat stateQuat_tr = {0.0f,0.0f,0.0f,1.0f};  // Orientation
static struct vec stateOmega_tr = {0.0f,0.0f,0.0f};       // Angular Rate [rad/s]



// POLICY VARIABLES
static float RREV_thr = 0.0f;
static float G1 = 0.0f;
static float G2 = 0.0f;

static bool policy_armed_flag = false;


// MOTOR VARIABLES
static uint32_t M1_pwm = 0; 
static uint32_t M2_pwm = 0; 
static uint32_t M3_pwm = 0; 
static uint32_t M4_pwm = 0; 

static float MS1 = 0;
static float MS2 = 0;
static float MS3 = 0;
static float MS4 = 0;

// TRAJECTORY VARIABLES
static float s_0 = 0.0f;
static float v = 0.0f;
static float a = 0.0f;
static float t = 0.0f;
static float T = 0.0f;
static uint8_t traj_type = 0;
static bool execute_traj = false;


void Controller::CMD_Callback(const crazyflie_msgs::RLCmd::ConstPtr &msg)
{
    setpoint.cmd_type = msg->cmd_type;
    setpoint.cmd_val1 = msg->cmd_vals.x;
    setpoint.cmd_val2 = msg->cmd_vals.y;
    setpoint.cmd_val3 = msg->cmd_vals.z;
    setpoint.cmd_flag = msg->cmd_flag;

    setpoint.GTC_cmd_rec = true;
}



void Controller::vicon_Callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    // Follow msg names from message details - "rqt -s rqt_msg" 
    
    // SET STATE VALUES INTO CLASS STATE VARIABLES
    _t = msg->header.stamp.toSec();
    _position = msg->pose.pose.position; 
    _velocity = msg->twist.twist.linear;

    state.position.x = _position.x;
    state.position.y = _position.y;
    state.position.z = _position.z;

    state.velocity.x = _velocity.x;
    state.velocity.y = _velocity.y;
    state.velocity.z = _velocity.z;

}

void Controller::imu_Callback(const sensor_msgs::Imu::ConstPtr &msg){
    _quaternion = msg->orientation;
    _omega = msg->angular_velocity;
    _accel = msg->linear_acceleration;

    state.attitudeQuaternion.x = _quaternion.x;
    state.attitudeQuaternion.y = _quaternion.y;
    state.attitudeQuaternion.z = _quaternion.z;
    state.attitudeQuaternion.w = _quaternion.w;

    sensorData.gyro.x = _omega.x;
    sensorData.gyro.y = _omega.y;
    sensorData.gyro.z = _omega.z;

    sensorData.acc.x = _accel.x;
    sensorData.acc.y = _accel.y;
    sensorData.acc.z = _accel.z;
    

}

void Controller::OF_Callback(const nav_msgs::Odometry::ConstPtr &msg){

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


void Controller::startController()
{
    ros::Rate rate(1000);
    
    while(ros::ok)
    {
        stateEstimator(&state, &sensorData, &control, tick);
        commanderGetSetpoint(&setpoint, &state);
        controllerGTC(&control, &setpoint, &sensorData, &state, tick, this);


        tick++;
        rate.sleep();
    }
}


void stateEstimator(state_t *state, sensorData_t *sensors, control_t *control, const uint32_t tick)
{

}

void commanderGetSetpoint(setpoint_t *setpoint, const state_t *state)
{

}

// EXPLICIT FUNTIONS

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


// Limit PWM value to accurate motor curve limit (60,000)
uint16_t limitPWM(int32_t value) 
{
  if(value > 60000)
  {
    value = 60000;
  }
  else if(value < 0)
  {
    value = 0;
  }

  return (uint16_t)value;
}


