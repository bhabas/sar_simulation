#ifdef __cplusplus // If C++ compiler then compile accordingly
extern "C" {
#endif





// ===========================================================
/**
 *
 * controller_gtc.h - Geometric Tracking Controller Interface
 *
 */

#ifndef __CONTROLLER_GTC_H__
#define __CONTROLLER_GTC_H__

#include "NN_Params/NN_Layers_Policy_WL.h"
#include "NN_Params/NN_Layers_Flip_WL.h"

// STANDARD LIBRARIES
#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>

// CF LIBARARIES
#include "math3d.h"
#include "log.h"
#include "param.h"
#include "debug.h"
#include "motors.h"

// CF HEADERS
#include "stabilizer_types.h"
#include "physicalConstants.h"
#include "quatcompress.h"
#include "nml.h"

#define PWM_MAX 60000
#define f_MAX (16.5)
#define g2Newton (9.81f/1000.0f)
#define Newton2g (1000.0f/9.81f)

typedef struct{
    nml_mat* mean;
    nml_mat* std;
}Scaler;

// FUNCTION PRIMITIVES
void controllerGTCInit(void);
bool controllerGTCTest(void);
void controllerGTCReset(void);
void controllerGTCTraj(void);
void controllerGTC(control_t *control, setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick);
void GTC_Command(setpoint_t *setpoint);

// NEURAL NETWORK PRIMITIVES
static void initNN_Layers(Scaler* scaler,nml_mat* W[], nml_mat* b[], char str[],int numLayers);
static float Sigmoid(float x);
static float Elu(float x);
static float NN_Forward_Policy(nml_mat* X, Scaler* scaler, nml_mat* W[], nml_mat* b[]);
static float NN_Forward_Flip(nml_mat* X, Scaler* scaler, nml_mat* W[], nml_mat* b[]);

// INIT PID VALUES BEFORE OVERWRITTEN BY CONFIG FILE
// XY POSITION PID
static float P_kp_xy = 0.5f;
static float P_kd_xy = 0.3f;
static float P_ki_xy = 0.1f;
static float i_range_xy = 0.3f;

// Z POSITION PID
static float P_kp_z = 1.2f;
static float P_kd_z = 0.35f;
static float P_ki_z = 0.1f;
static float i_range_z = 0.25f;

// XY ATTITUDE PID
static float R_kp_xy = 0.004f;
static float R_kd_xy = 0.0017f;
static float R_ki_xy = 0.0f;
static float i_range_R_xy = 1.0f;

// Z ATTITUDE PID
static float R_kp_z = 0.003f;
static float R_kd_z = 0.001f;
static float R_ki_z = 0.002;
static float i_range_R_z = 0.5f;



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
static float Tau = 0.0f;    // [s]
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
static uint32_t M1_pwm = 0; 
static uint32_t M2_pwm = 0; 
static uint32_t M3_pwm = 0; 
static uint32_t M4_pwm = 0; 

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
static bool safeModeEnable = true;


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


static float F_thrust_flip = 0.0f; // [g]
static float M_x_flip = 0.0f;      // [N*m]
static float M_y_flip = 0.0f;      // [N*m]
static float M_z_flip = 0.0f;      // [N*m]



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

static void initNN_Layers(Scaler* scaler,nml_mat* W[], nml_mat* b[], char str[],int numLayers)
{
    char* array_token;
    char* save_ptr;

    array_token = strtok_r(str,"*",&save_ptr);
    scaler->mean = nml_mat_fromstr(array_token);
    array_token = strtok_r(NULL,"*",&save_ptr);

    scaler->std = nml_mat_fromstr(array_token);
    array_token = strtok_r(NULL,"*",&save_ptr);


    for (int i = 0; i < numLayers; i++)
    {
        W[i] = nml_mat_fromstr(array_token);
        array_token = strtok_r(NULL,"*",&save_ptr);
        b[i] = nml_mat_fromstr(array_token);
        array_token = strtok_r(NULL,"*",&save_ptr);
    }


}

static float NN_Forward_Policy(nml_mat* X, Scaler* scaler, nml_mat* W[], nml_mat* b[])
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

static float NN_Forward_Flip(nml_mat* X, Scaler* scaler, nml_mat* W[], nml_mat* b[])
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




    // // SAVE OUTPUT VALUE
    float y_output = a3->data[0][0];


    // // FREE MATRICES FROM STACK
    nml_mat_free(X_input);
    nml_mat_free(WX1);
    nml_mat_free(WX2);
    nml_mat_free(WX3);

    nml_mat_free(a1);
    nml_mat_free(a2);
    nml_mat_free(a3);

    return y_output;
}

static float Sigmoid(float x)
{
    return 1/(1+exp(-x));
}

static float Elu(float x)
{
    if(x>0) return x;

    else return exp(x)-1.0f;
 
}

// Limit PWM value to accurate motor curve limit (60,000)
static uint16_t limitPWM(int32_t value) 
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



// // ==================================
// //         Logging Compression
// // ==================================

// static struct {
//     // Compressed positions [mm]
//     uint32_t xy; 
//     int16_t z;

//     // Compressed velocities [mm/s]
//     uint32_t vxy; 
//     int16_t vz;

//     // compressed quaternion, see quatcompress.h
//     int32_t quat; 

//     // Compressed angular velocity [milli-rad/sec]
//     uint32_t wxy; 
//     int16_t wz;

//     // Compressed actuation states
//     uint32_t Mxy;   // [N*um]
//     uint32_t FMz;   // [mN | N*um]

//     uint32_t MS_PWM12; 
//     uint32_t MS_PWM34;
    
//     // Compressed Optical Flow Values
//     uint32_t OF_xy; // [milli-rad/s]
//     int16_t RREV;   // [milli-rad/s]
//     int16_t d_ceil; // [mm]

//     uint32_t NN_FP; // NN_flip,NN_policy

// } StatesZ_GTC;


// static struct {
    
//     uint32_t xy;  // Compressed position [mm]
//     int16_t z;

//     uint32_t vxy; // Compressed velocities [mm/s]
//     int16_t vz;

//     uint32_t axy; // Compress accelerations [mm/s^2]
//     int16_t az;

// } setpointZ_GTC;


// static struct {

//     // Compressed positions [mm]
//     uint32_t xy; 
//     int16_t z;

//     // Compressed velocities [mm/s]
//     uint32_t vxy; 
//     int16_t vz;

//     // compressed quaternion, see quatcompress.h
//     int32_t quat; 

//     // Compressed angular velocity [milli-rad/sec]
//     uint32_t wxy; 
//     int16_t wz;

//     // Compressed Optical Flow Values
//     uint32_t OF_xy; // [milli-rad/s]
//     int16_t RREV;   // [milli-rad/s]
//     int16_t d_ceil; // [m]

//     uint32_t NN_FP; // NN_flip,NN_policy

// } FlipStatesZ_GTC;


// static void compressStates(){
//     StatesZ_GTC.xy = compressXY(statePos.x,statePos.y);
//     StatesZ_GTC.z = (int16_t)(statePos.z * 1000.0f);

//     StatesZ_GTC.vxy = compressXY(stateVel.x, stateVel.y);
//     StatesZ_GTC.vz = (int16_t)(stateVel.z * 1000.0f);

//     StatesZ_GTC.wxy = compressXY(stateOmega.x,stateOmega.y);
//     StatesZ_GTC.wz = (int16_t)(stateOmega.z * 1000.0f);


//     float const q[4] = {
//         stateQuat.x,
//         stateQuat.y,
//         stateQuat.z,
//         stateQuat.w};
//     StatesZ_GTC.quat = quatcompress(q);

//     // COMPRESS SENSORY VALUES
//     StatesZ_GTC.OF_xy = compressXY(OF_x,OF_y);
//     StatesZ_GTC.RREV = (int16_t)(RREV * 1000.0f); 
//     StatesZ_GTC.d_ceil = (int16_t)(d_ceil * 1000.0f);

//     // COMPRESS THRUST/MOMENT VALUES
//     StatesZ_GTC.FMz = compressXY(F_thrust,M.z*1000.0f);
//     StatesZ_GTC.Mxy = compressXY(M.x*1000.0f,M.y*1000.0f);

//     // COMPRESS PWM VALUES
//     StatesZ_GTC.MS_PWM12 = compressXY(M1_pwm*0.5e-3f,M2_pwm*0.5e-3f);
//     StatesZ_GTC.MS_PWM34 = compressXY(M3_pwm*0.5e-3f,M4_pwm*0.5e-3f);

//     StatesZ_GTC.NN_FP = compressXY(NN_flip,NN_policy);

// }




// static void compressSetpoints(){
//     setpointZ_GTC.xy = compressXY(x_d.x,x_d.y);
//     setpointZ_GTC.z = (int16_t)(x_d.z * 1000.0f);

//     setpointZ_GTC.vxy = compressXY(v_d.x,v_d.y);
//     setpointZ_GTC.vz = (int16_t)(v_d.z * 1000.0f);

//     setpointZ_GTC.axy = compressXY(a_d.x,a_d.y);
//     setpointZ_GTC.az = (int16_t)(a_d.z * 1000.0f);
// }


// static void compressFlipStates(){
//     FlipStatesZ_GTC.xy = compressXY(statePos_tr.x,statePos_tr.y);
//     FlipStatesZ_GTC.z = (int16_t)(statePos_tr.z * 1000.0f);

//     FlipStatesZ_GTC.vxy = compressXY(stateVel_tr.x, stateVel_tr.y);
//     FlipStatesZ_GTC.vz = (int16_t)(stateVel_tr.z * 1000.0f);

//     FlipStatesZ_GTC.wxy = compressXY(stateOmega_tr.x,stateOmega_tr.y);
//     FlipStatesZ_GTC.wz = (int16_t)(stateOmega_tr.z * 1000.0f);


//     float const q[4] = {
//         stateQuat_tr.x,
//         stateQuat_tr.y,
//         stateQuat_tr.z,
//         stateQuat_tr.w};
//     FlipStatesZ_GTC.quat = quatcompress(q);

//    FlipStatesZ_GTC.OF_xy = compressXY(OF_x_tr,OF_y_tr);
//    FlipStatesZ_GTC.RREV = (int16_t)(RREV_tr * 1000.0f); 
//    FlipStatesZ_GTC.d_ceil = (int16_t)(d_ceil_tr * 1000.0f);

//    FlipStatesZ_GTC.NN_FP = compressXY(NN_tr_flip,NN_tr_policy);

// }





#endif //__CONTROLLER_GTC_H__


// ===========================================================









#ifdef __cplusplus
}
#endif