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
#include "crazyflie_msgs/MS.h"


#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "gazebo_msgs/SetPhysicsProperties.h"


// STANDARD LIBRARIES
#include <math.h>       
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <pwd.h>


// CF LIBRARIES
#include "math3d.h"
#include "stabilizer_types.h"
#include "nml.h"

#include "NN_Params/NN_Layers_Policy_Wide-Long.h"
#include "NN_Params/NN_Layers_Flip_Wide-Long.h"

#define PWM_MAX 60000 // Limit PWM to give buffer for battery drop
#define f_MAX (16.5) // Max motor thrust [g]
#define g2Newton (9.81f/1000.0f)
#define Newton2g (1000.0f/9.81f)

// =======================
//       C++ CODE
// =======================
using namespace std;
class Controller;
// =======================

typedef struct{
    nml_mat* mean;
    nml_mat* std;
}Scaler;

// FIRMWARE PRIMITIVES
void stateEstimator(state_t *state, sensorData_t *sensors, control_t *control, const uint32_t tick);
void commanderGetSetpoint(setpoint_t *setpoint, const state_t *state);


// GTC PRIMITIVES
void controllerGTCInit(void);
bool controllerGTCTest(void);
void controllerGTCReset(Controller* _CTRL);
void controllerGTCTraj(void);
void controllerGTC(control_t *control, setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick,
                                         const flowDeck_t *flowDeck,
                                         Controller* _CTRL);
void GTC_Command(setpoint_t *setpoint, Controller* _CTRL);

// NEURAL NETWORK PRIMITIVES
void initScaler(Scaler* scaler,char path[]);
void NN_Scale(nml_mat* X, Scaler* scaler);
void initNN_Layers(Scaler* scaler,nml_mat* W[], nml_mat* b[], char str[],int numLayers);
float Sigmoid(float x);
float Elu(float x);
float NN_Policy(nml_mat* X, nml_mat* W[], nml_mat* b[]);
float NN_Flip(nml_mat* X, nml_mat* W[], nml_mat* b[]);



// INIT PID VALUES BEFORE OVERWRITTEN BY CONFIG FILE
// XY POSITION PID
float P_kp_xy = 0.3f;
float P_kd_xy = 0.01f;
float P_ki_xy = 0.0f;
float i_range_xy = 0.3f;

// Z POSITION PID
float P_kp_z = 1.20f;
float P_kd_z = 0.35f;
float P_ki_z = 0.0f;
float i_range_z = 0.25f;

// XY ATTITUDE PID
float R_kp_xy = 0.02f;
float R_kd_xy = 0.08f;
float R_ki_xy = 0.0f;
float i_range_R_xy = 1.0f;

// Z ATTITUDE PID
float R_kp_z = 0.003f;
float R_kd_z = 0.025f;
float R_ki_z = 0.000f;
float i_range_R_z = 0.5f;



// SYSTEM PARAMETERS
static float m = 0.0376; // [g]
static float g = 9.81f;
struct mat33 J; // Rotational Inertia Matrix [kg*m^2]
static float h_ceiling = 2.10f; // [m]

static float d = 0.040f;    // COM to Prop [m]
static float dp = 0.028284; // COM to Prop along x-axis [m]
                            // [dp = d*sin(45 deg)]

static float const kf = 2.2e-8f;    // Thrust Coeff [N/(rad/s)^2]
static float const c_tf = 0.00618f; // Moment Coeff [Nm/N]

// INIT STATE VALUES
static struct vec statePos = {0.0f,0.0f,0.0f};         // Pos [m]
static struct vec stateVel = {0.0f,0.0f,0.0f};         // Vel [m/s]
static struct quat stateQuat = {0.0f,0.0f,0.0f,1.0f};  // Orientation
static struct vec stateOmega = {0.0f,0.0f,0.0f};       // Angular Rate [rad/s]

static struct mat33 R; // Orientation as rotation matrix
static struct vec stateEul = {0.0f,0.0f,0.0f}; // Pose in Euler Angles [YZX Notation]

// OPTICAL FLOW STATES
static float RREV = 0.0f;   // [rad/s]
static float OF_x = 0.0f;   // [rad/s]
static float OF_y = 0.0f;   // [rad/s] 
static float d_ceil = 0.0f;


// INIT DESIRED STATES
static struct vec x_d = {0.0f,0.0f,0.4f}; // Pos-desired [m]
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

// STATE ERRORS
static struct vec e_x;  // Pos-error [m]
static struct vec e_v;  // Vel-error [m/s]
static struct vec e_PI; // Pos. Integral-error [m*s]

static struct vec e_R;  // Rotation-error [rad]
static struct vec e_w;  // Omega-error [rad/s]
static struct vec e_RI; // Rot. Integral-error [rad*s]


// TEMPORARY CALC VECS/MATRICES
static struct vec temp1_v; 
static struct vec temp2_v;
static struct vec temp3_v;
static struct vec temp4_v;
static struct mat33 temp1_m;  

static struct vec P_effort; // Effort by positional PID
static struct vec R_effort; // Effor by rotational PID

static struct mat33 RdT_R; // Rd' * R
static struct mat33 RT_Rd; // R' * Rd
static struct vec Gyro_dyn;

// CONTROLLER ACTUATIONS
static struct vec F_thrust_ideal;           // Ideal thrust vector [N]
static struct vec M_d = {0.0f,0.0f,0.0f};   // Desired moment [N*m]

static float F_thrust = 0.0f;               // Implemented body thrust [N]
static struct vec M = {0.0f,0.0f,0.0f};     // Implemented body moments [N*m]


// MOTOR THRUSTS
static float f_thrust_g; // Motor thrust - Thrust [g]
static float f_roll_g;   // Motor thrust - Roll   [g]
static float f_pitch_g;  // Motor thrust - Pitch  [g]
static float f_yaw_g;    // Motor thrust - Yaw    [g]


// MOTOR VARIABLES
static uint16_t M1_pwm = 0; 
static uint16_t M2_pwm = 0; 
static uint16_t M3_pwm = 0; 
static uint16_t M4_pwm = 0; 

static uint16_t MS1 = 0;
static uint16_t MS2 = 0;
static uint16_t MS3 = 0;
static uint16_t MS4 = 0;



// INIT CTRL GAIN VECTORS 
static struct vec Kp_p; // Pos. Proportional Gains 
static struct vec Kd_p; // Pos. Derivative Gains
static struct vec Ki_p; // Pos. Integral Gains  

static struct vec Kp_R; // Rot. Proportional Gains
static struct vec Kd_R; // Rot. Derivative Gains
static struct vec Ki_R; // Rot. Integral Gains

// CONTROLLER GAIN FLAGS
static float kp_xf = 1; // Pos. Gain Flag
static float kd_xf = 1; // Pos. Derivative Gain Flag
// float ki_xf = 1; // Pos. Integral Flag
// float kp_Rf = 1; // Rot. Gain Flag
// float kd_Rf = 1; // Rot. Derivative Gain Flag
// float ki_Rf = 1; // Rot. Integral Flag

static float dt = (float)(1.0f/RATE_500_HZ);

// CONTROLLER FLAGS
static bool tumbled = false;
static bool tumble_detection = true;
static bool motorstop_flag = false;
static bool errorReset = false; // Resets error vectors (removed integral windup)

static bool execute_traj = false;
static bool policy_armed_flag = false;

static bool flip_flag = false;
static bool onceFlag = false;

static bool Moment_flag = false;
static bool attCtrlEnable = false;


// POLICY VARIABLES
static float RREV_thr = 0.0f;
static float G1 = 0.0f;
static float G2 = 0.0f;

// DEFINE POLICY TYPE ACTIVATED
typedef enum {
    RL = 0, // Reinforcement Learning
    NN = 1  // Neural Network
} Policy_Type;
static Policy_Type POLICY_TYPE = RL; // Default to RL



// STATE VALUES AT FLIP TRIGGER
static struct vec statePos_tr = {0.0f,0.0f,0.0f};         // Pos [m]
static struct vec stateVel_tr = {0.0f,0.0f,0.0f};         // Vel [m/s]
static struct quat stateQuat_tr = {0.0f,0.0f,0.0f,1.0f};  // Orientation
static struct vec stateOmega_tr = {0.0f,0.0f,0.0f};       // Angular Rate [rad/s]

static float RREV_tr = 0.0f; // [rad/s]
static float OF_x_tr = 0.0f; // [rad/s]
static float OF_y_tr = 0.0f; // [rad/s]
static float d_ceil_tr = 0.0f; // [m/s]


float F_thrust_flip = 0.0f; // [g]
float M_x_flip = 0.0f;      // [N*m]
float M_y_flip = 0.0f;      // [N*m]
float M_z_flip = 0.0f;      // [N*m]



// TRAJECTORY VARIABLES (1-D | Z-Axis)
static float s_0 = 0.0f;
static float v = 0.0f;
static float a = 0.0f;

typedef enum {
    x = 0, 
    y = 1,
    z = 2
} axis_direction;

static axis_direction traj_type;

static struct vec s_0_t = {0.0f, 0.0f, 0.0f};
static struct vec v_t = {0.0f, 0.0f, 0.0f};
static struct vec a_t = {0.0f, 0.0f, 0.0f};
static struct vec T = {0.0f, 0.0f, 0.0f};
static struct vec t_traj = {0.0f, 0.0f, 0.0f};

// NEURAL NETWORK INITIALIZATION
static Scaler Scaler_Flip;
static Scaler Scaler_Policy;

static nml_mat* X; // STATE MATRIX TO BE INPUT INTO NN


static nml_mat* W_policy[3];
static nml_mat* b_policy[3];

static nml_mat* W_flip[3];
static nml_mat* b_flip[3];

static float NN_flip = 0.0f;       // NN output value for flip classification
static float NN_policy = 0.0f;     // NN output value for policy My

static float NN_tr_flip = 0.0f;    // NN value at flip trigger
static float NN_tr_policy = 0.0f;  // NN policy value at flip trigger



void stateEstimator(state_t *state, sensorData_t *sensors, control_t *control, const uint32_t tick)
{

}

void commanderGetSetpoint(setpoint_t *setpoint, const state_t *state)
{

}

void initNN_Layers(Scaler* scaler,nml_mat* W[], nml_mat* b[], char str[],int numLayers)
{
    char array_list[2048];
    char* array_token;
    char* save_ptr;

    strcpy(array_list,str);


    array_token = strtok_r(array_list,"*",&save_ptr);
    // printf("%s\n",array_token);
    scaler->mean = nml_mat_fromstr(array_token);
    array_token = strtok_r(NULL,"*",&save_ptr);

    scaler->std = nml_mat_fromstr(array_token);
    array_token = strtok_r(NULL,"*",&save_ptr);


    nml_mat_print(scaler->mean);
    nml_mat_print(scaler->std);

    for (int i = 0; i < numLayers; i++)
    {
        W[i] = nml_mat_fromstr(array_token);
        array_token = strtok_r(NULL,"*",&save_ptr);
        b[i] = nml_mat_fromstr(array_token);
        array_token = strtok_r(NULL,"*",&save_ptr);
    }

}

float NN_Policy(nml_mat* X, Scaler* scaler, nml_mat* W[], nml_mat* b[])
{   
    nml_mat* X_input = nml_mat_cp(X);

    // X_input = nml_mat_divEl(nml_mat_subEl(X-scaler->mean),scaler->std);
    for(int i=0;i<3;i++)
    {
        X_input->data[i][0] = (X->data[i][0] - scaler->mean->data[i][0])/scaler->std->data[i][0];
    }

    //LAYER 1
    //Sigmoid(W*X+b)
    nml_mat *WX1 = nml_mat_dot(W[0],X_input); 
    nml_mat_add_r(WX1,b[0]);
    nml_mat *a1 = nml_mat_funcElement(WX1,Sigmoid);

    // LAYER 2
    // Sigmoid(W*X+b)
    nml_mat *WX2 = nml_mat_dot(W[1],a1); 
    nml_mat_add_r(WX2,b[1]);
    nml_mat *a2 = nml_mat_funcElement(WX2,Sigmoid);

    // LAYER 3
    // (W*X+b)
    nml_mat *WX3 = nml_mat_dot(W[2],a2); 
    nml_mat_add_r(WX3,b[2]);
    nml_mat *a3 = nml_mat_cp(WX3);


    // SAVE OUTPUT VALUE
    float y_output = (float)a3->data[0][0];

    // FREE MATRICES FROM STACK
    nml_mat_free(X_input);
    nml_mat_free(WX1);
    nml_mat_free(WX2);
    nml_mat_free(WX3);

    nml_mat_free(a1);
    nml_mat_free(a2);
    nml_mat_free(a3);


    return y_output;
}

float NN_Flip(nml_mat* X, Scaler* scaler, nml_mat* W[], nml_mat* b[])
{
    nml_mat* X_input = nml_mat_cp(X);
    for(int i=0;i<3;i++)
    {
        X_input->data[i][0] = (X->data[i][0] - scaler->mean->data[i][0])/scaler->std->data[i][0];
    }

    // LAYER 1
    // Elu(W*X+b)
    nml_mat *WX1 = nml_mat_dot(W[0],X_input); 
    nml_mat_add_r(WX1,b[0]);
    nml_mat *a1 = nml_mat_funcElement(WX1,Elu);


    // LAYER 2
    // Elu(W*X+b)
    nml_mat *WX2 = nml_mat_dot(W[1],a1); 
    nml_mat_add_r(WX2,b[1]);
    nml_mat *a2 = nml_mat_funcElement(WX2,Elu);


    // LAYER 3
    // Sigmoid(W*X+b)
    nml_mat *WX3 = nml_mat_dot(W[2],a2); 
    nml_mat_add_r(WX3,b[2]);
    nml_mat *a3 = nml_mat_funcElement(WX3,Sigmoid);


    // SAVE OUTPUT VALUE
    float y_output = a3->data[0][0];


    // FREE MATRICES FROM STACK
    nml_mat_free(X_input);
    nml_mat_free(WX1);
    nml_mat_free(WX2);
    nml_mat_free(WX3);

    nml_mat_free(a1);
    nml_mat_free(a2);
    nml_mat_free(a3);


    return y_output;
}

float Sigmoid(float x)
{
    return 1/(1+exp(-x));
}

float Elu(float x)
{
    if(x>0) return x;

    else return exp(x)-1.0f;
 
}

// Converts thrust in grams to their respective PWM values
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

// Converts thrust in PWM to their respective values in grams
static inline float PWM2thrust(int32_t M_PWM) 
{
    // Conversion values calculated from PWM to Thrust Curve
    float a = 3.31e4;
    float b = 1.12e1;
    float c = 8.72;
    float d = 3.26e4;

    float f = b*atan2f(M_PWM-d,a)+c;

    if(f<0)
    {
      f = 0;
    }

    return f;
}


// Limit PWM value to accurate motor curve limit (60,000)
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

// ======================================================== 
//            ___  _     _      ___ ___   __| | ___ 
//           / __|| |_ _| |_   / __/ _ \ / _` |/ _ \
//          | (_|_   _|_   _| | (_| (_) | (_| |  __/
//           \___||_|   |_|    \___\___/ \__,_|\___|
//                                                 
// ========================================================

class Controller
{
    public:
        // CONSTRUCTOR TO START PUBLISHERS AND SUBSCRIBERS (Similar to Python's __init__() )
        Controller(ros::NodeHandle *nh){
            
            // BODY SENSORS
            _IMU_Subscriber = nh->subscribe("/cf1/imu",1,&Controller::imu_Callback,this,ros::TransportHints().tcpNoDelay());
            _OF_Subscriber = nh->subscribe("/cf1/OF_sensor",1,&Controller::OF_Callback,this,ros::TransportHints().tcpNoDelay()); 

            // COMMANDS AND INFO
            _CMD_Subscriber = nh->subscribe("/rl_ctrl",50,&Controller::CMD_Callback,this,ros::TransportHints().tcpNoDelay());
            _CTRL_Publisher = nh->advertise<crazyflie_msgs::CtrlData>("/ctrl_data",1);
            _MS_Publisher = nh->advertise<crazyflie_msgs::MS>("/MS",1);
            _SimSpeed_Client = nh->serviceClient<gazebo_msgs::SetPhysicsProperties>("/gazebo/set_physics_properties");

            // ENVIRONMENT SENSORS
            _viconState_Subscriber = nh->subscribe("/env/vicon_state",1,&Controller::vicon_Callback,this,ros::TransportHints().tcpNoDelay());
            _CeilingFT_Subcriber = nh->subscribe("/env/ceiling_force_sensor",5,&Controller::ceilingFT_Callback,this,ros::TransportHints().tcpNoDelay());
            _RLData_Subscriber = nh->subscribe("/rl_data",5,&Controller::RLData_Callback,this,ros::TransportHints().tcpNoDelay());


            // SIMULATION SETTINGS FROM CONFIG FILE
            ros::param::get("/MODEL_NAME",_MODEL_NAME);
            ros::param::get("/CEILING_HEIGHT",_H_CEILING);
            ros::param::get("/CF_MASS",_CF_MASS);
            ros::param::get("/POLICY_TYPE",_POLICY_TYPE);
            POLICY_TYPE = (Policy_Type)_POLICY_TYPE; // Cast ROS param (int) to enum (Policy_Type)

            // DEBUG SETTINGS
            ros::param::get("/SIM_SPEED",_SIM_SPEED);
            ros::param::get("/CTRL_DEBUG_SLOWDOWN", _CTRL_DEBUG_SLOWDOWN);
            ros::param::get("/LANDING_SLOWDOWN",_LANDING_SLOWDOWN_FLAG);
            ros::param::get("/SIM_SLOWDOWN_SPEED",_SIM_SLOWDOWN_SPEED);

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

            // INITIALIZE STATE AND SENSOR VALUES
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


            // INITIALIZE SETPOINT VALUES
            setpoint.position.x = 0.0f;
            setpoint.position.y = 0.0f;
            setpoint.position.z = 0.4f;

            setpoint.velocity.x = 0.0f;
            setpoint.velocity.y = 0.0f;
            setpoint.velocity.z = 0.0f;

            setpoint.acceleration.x = 0.0f;
            setpoint.acceleration.y = 0.0f;
            setpoint.acceleration.z = 0.0f;

            // INITIALIZE COMMAND VALUES
            setpoint.cmd_type = 101;
            setpoint.cmd_val1 = 0.0f;
            setpoint.cmd_val2 = 0.0f;
            setpoint.cmd_val3 = 0.0f;
            setpoint.cmd_flag = 0.0f;


            // START CONTROLLER
            // controllerThread = std::thread(&Controller::startController, this);           
        }

        // DEFINE FUNCTION PROTOTYPES
        void startController();
        void vicon_Callback(const nav_msgs::Odometry::ConstPtr &msg);
        void imu_Callback(const sensor_msgs::Imu::ConstPtr &msg);
        void OF_Callback(const nav_msgs::Odometry::ConstPtr &msg);   
        void CMD_Callback(const crazyflie_msgs::RLCmd::ConstPtr &msg);
        void ceilingFT_Callback(const crazyflie_msgs::ImpactData::ConstPtr &msg);
        void adjustSimSpeed(float speed_mult);
        void RLData_Callback(const crazyflie_msgs::RLData::ConstPtr &msg);
        




        crazyflie_msgs::MS _MS_msg;
        crazyflie_msgs::CtrlData _ctrl_msg;

        ros::Publisher _MS_Publisher;
        ros::Publisher _CTRL_Publisher;

        ros::ServiceClient _SimSpeed_Client;
        ros::Subscriber _CeilingFT_Subcriber;
        ros::Subscriber _RLData_Subscriber;




        // SENSORS
        ros::Subscriber _viconState_Subscriber;
        ros::Subscriber _IMU_Subscriber;
        ros::Subscriber _OF_Subscriber;
        ros::Subscriber _CMD_Subscriber;

        // INITIALIZE ROS MSG VARIABLES
        geometry_msgs::Point _position; 
        geometry_msgs::Vector3 _velocity;
        geometry_msgs::Quaternion _quaternion;
        geometry_msgs::Vector3 _omega;
        geometry_msgs::Vector3 _accel;

        float _t;
        ros::Time _t_flip;


        // ROS SPECIFIC VALUES
        int _impact_flag = 0;
        int _slowdown_type = 0;
        float _H_CEILING = 2.10;
        bool _LANDING_SLOWDOWN_FLAG;
        float _SIM_SPEED; 
        float _SIM_SLOWDOWN_SPEED;
        float _CF_MASS;
        int _CTRL_DEBUG_SLOWDOWN;
        int _POLICY_TYPE;
        string _MODEL_NAME;
        
        

        float _RREV = 0.0f;
        float _OF_x = 0.0f;
        float _OF_y = 0.0f;
        float _d_ceil = 0.0f;




    private:

        // DEFINE THREAD OBJECTS
        std::thread controllerThread;


        // DATA STRUCTS EQUIVALENT TO THOSE IN CF FIRMWARE
        control_t control;
        setpoint_t setpoint;
        sensorData_t sensorData;
        state_t state;
        uint32_t tick = 0;
        flowDeck_t flowDeck;
        


};


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
    float d_ceil = _H_CEILING-position.z; // h_ceiling - height    

    // SET SENSOR VALUES INTO CLASS VARIABLES
    // _RREV = msg->RREV;
    // _OF_x = msg->OF_x;
    // _OF_y = msg->OF_y;


    flowDeck.dist = d_ceil;
    flowDeck.RREV = velocity.z/d_ceil;
    flowDeck.OF_y = -velocity.x/d_ceil;
    flowDeck.OF_x = -velocity.y/d_ceil;
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

void Controller::ceilingFT_Callback(const crazyflie_msgs::ImpactData::ConstPtr &msg)
{
    _impact_flag = msg->impact_flag;
}

void Controller::RLData_Callback(const crazyflie_msgs::RLData::ConstPtr &msg){

    if (msg->reset_flag == true){

        controllerGTCReset(this);

    }
}

void Controller::startController() // MAIN CONTROLLER LOOP
{
    ros::Rate rate(RATE_MAIN_LOOP);
    controllerGTCInit();
   
    while(ros::ok)
    {
        stateEstimator(&state, &sensorData, &control, tick);
        commanderGetSetpoint(&setpoint, &state);
        controllerGTC(&control, &setpoint, &sensorData, &state, tick, &flowDeck, this);

        tick++;
        rate.sleep();
    }
}
