#ifdef __cplusplus // If C++ compiler then compile accordingly
extern "C" {
#endif


// CRAZYFLIE FIRMWARE CODE BELOW
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

// FUNCTION PRIMITIVES
void controllerGTCInit(void);
bool controllerGTCTest(void);
void controllerGTCReset(void);
void controllerGTCTraj(void);
void controllerGTC(control_t *control, setpoint_t *setpoint,
                                         sensorData_t *sensors,
                                         state_t *state,
                                         const uint32_t tick);
void controlOutput(state_t *state, sensorData_t *sensors);
void GTC_Command(setpoint_t *setpoint);

// =================================
//    CONTROL GAIN INITIALIZATION
// =================================
// XY POSITION PID
extern float P_kp_xy;
extern float P_kd_xy;
extern float P_ki_xy;
extern float i_range_xy;

// Z POSITION PID
extern float P_kp_z;
extern float P_kd_z;
extern float P_ki_z;
extern float i_range_z;

// XY ATTITUDE PID
extern float R_kp_xy;
extern float R_kd_xy;
extern float R_ki_xy;
extern float i_range_R_xy;

// Z ATTITUDE PID
extern float R_kp_z;
extern float R_kd_z;
extern float R_ki_z;
extern float i_range_R_z;

// INIT CTRL GAIN VECTORS 
extern struct vec Kp_p; // Pos. Proportional Gains 
extern struct vec Kd_p; // Pos. Derivative Gains
extern struct vec Ki_p; // Pos. Integral Gains  

extern struct vec Kp_R; // Rot. Proportional Gains
extern struct vec Kd_R; // Rot. Derivative Gains
extern struct vec Ki_R; // Rot. Integral Gains

// CONTROLLER GAIN FLAGS
extern float kp_xf; // Pos. Gain Flag
extern float kd_xf; // Pos. Derivative Gain Flag

// SYSTEM PARAMETERS
extern float m;
extern float h_ceiling;

// INIT STATE VALUES
extern struct vec statePos;
extern struct vec stateVel;
extern struct quat stateQuat;
extern struct vec stateOmega;
extern struct vec stateEul;

// OPTICAL FLOW STATES
extern float Tau;   // [s]
extern float OFx;   // [rad/s]
extern float OFy;   // [rad/s] 
extern float RREV;  // [rad/s]
extern float d_ceil;

// INIT DESIRED STATES
extern struct vec x_d;      // Pos-desired [m]
extern struct vec v_d;      // Vel-desired [m/s]
extern struct vec a_d;      // Acc-desired [m/s^2]

extern struct quat quat_d;  // Orientation-desired [qx,qy,qz,qw]
extern struct vec eul_d;    // Euler Angle-desired [rad? deg? TBD]
extern struct vec omega_d;  // Omega-desired [rad/s]
extern struct vec domega_d; // Ang. Acc-desired [rad/s^2]

extern struct vec M_d;             // Desired moment [N*m]

// STATE ERRORS
extern struct vec e_x;  // Pos-error [m]
extern struct vec e_v;  // Vel-error [m/s]
extern struct vec e_PI; // Pos. Integral-error [m*s]

extern struct vec e_R;  // Rotation-error [rad]
extern struct vec e_w;  // Omega-error [rad/s]
extern struct vec e_RI; // Rot. Integral-error [rad*s]

// CONTROLLER ACTUATIONS
struct vec F_thrust_ideal;  // Ideal thrust vector [N]
extern float F_thrust;      // Implemented body thrust [N]
extern struct vec M;        // Implemented body moments [N*m]

// MOTOR THRUSTS
extern float f_thrust_g; // Motor thrust - Thrust [g]
extern float f_roll_g;   // Motor thrust - Roll   [g]
extern float f_pitch_g;  // Motor thrust - Pitch  [g]
extern float f_yaw_g;    // Motor thrust - Yaw    [g]

// MOTOR VARIABLES
extern uint16_t M1_pwm; 
extern uint16_t M2_pwm; 
extern uint16_t M3_pwm; 
extern uint16_t M4_pwm; 

// =================================
//  FLAGS AND SYSTEM INITIALIZATION
// =================================

// CONTROLLER FLAGS
extern bool tumbled;
extern bool tumble_detection;
extern bool motorstop_flag;
extern bool errorReset; // Resets error vectors (removed integral windup)
extern bool safeModeFlag;

extern bool execute_traj;
extern bool policy_armed_flag;

extern bool flip_flag;
extern bool onceFlag;

extern bool moment_flag;
extern bool attCtrlEnable;
extern bool safeModeEnable;

// ======================================
//  RECORD SYSTEM STATES AT FLIP TRIGGER
// ======================================

// CARTESIAN STATES
extern struct vec statePos_tr;      // Pos [m]
extern struct vec stateVel_tr;      // Vel [m/s]
extern struct quat stateQuat_tr;    // Orientation
extern struct vec stateOmega_tr;    // Angular Rate [rad/s]

// OPTICAL FLOW STATES
extern float Tau_tr;        // [rad/s]
extern float OFx_tr;        // [rad/s]
extern float OFy_tr;        // [rad/s]
extern float RREV_tr;       // [rad/s]
extern float d_ceil_tr;     // [m/s]

// CONTROLLER STATES
extern float F_thrust_flip; // [N]
extern float M_x_flip;      // [N*m]
extern float M_y_flip;      // [N*m]
extern float M_z_flip;      // [N*m]

// ==========================
//  RL POLICY INITIALIZATION
// ==========================
extern float RREV_thr;  // RREV trigger
extern float G1;        // Body moment value
extern float G2;        // Deprecated state value

// ===============================
//  NEURAL NETWORK INITIALIZATION
// ===============================

// NN OUTPUTS
extern float NN_flip;           // NN output value for flip classification
extern float NN_policy;         // NN output value for policy My

// NN OUTPUTS AT FLIP TRIGGER
extern float NN_tr_flip;        // NN value at flip trigger
extern float NN_tr_policy;      // NN policy value at flip trigger

typedef enum {
    RL = 0, // Reinforcement Learning
    NN = 1  // Neural Network
} Policy_Type;
extern Policy_Type POLICY_TYPE;



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
static int32_t thrust2PWM(float f) 
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
static float PWM2thrust(int32_t M_PWM) 
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



#endif //__CONTROLLER_GTC_H__


// ===========================================================






#define consolePrintf printf


#ifdef __cplusplus
}
#endif