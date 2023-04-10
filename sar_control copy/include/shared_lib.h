#ifndef SHARED_LIB_H
#define SHARED_LIB_H

// STANDARD LIBRARIES
#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>


// CF LIBARARIES
#include "math3d.h"
#include "stabilizer_types.h"


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
extern float Ixx;
extern float Iyy;
extern float Izz;

// INIT STATE VALUES
extern struct vec statePos;
extern struct vec stateVel;
extern struct quat stateQuat;
extern struct vec stateOmega;
extern struct vec stateEul;

// OPTICAL FLOW STATES
extern float Tau;   // [s]
extern float Theta_x;   // [rad/s] 
extern float Theta_y;   // [rad/s]
extern float D_perp;

// ESTIMATED OPTICAL FLOW STATES
extern float Tau_est; // [s]
extern float Theta_x_est; // [rad/s]
extern float Theta_y_est; // [rad/s]

// INIT DESIRED STATES
extern struct vec x_d;      // Pos-desired [m]
extern struct vec v_d;      // Vel-desired [m/s]
extern struct vec a_d;      // Acc-desired [m/s^2]
extern struct vec b1_d;

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

// MOTOR THRUST ACTIONS
extern float f_thrust_g;    // Motor thrust - Thrust [g]
extern float f_roll_g;      // Motor thrust - Roll   [g]
extern float f_pitch_g;     // Motor thrust - Pitch  [g]
extern float f_yaw_g;       // Motor thrust - Yaw    [g]

// INDIVIDUAL MOTOR THRUSTS
extern float M1_thrust;
extern float M2_thrust;
extern float M3_thrust;
extern float M4_thrust;

// MOTOR PWM VALUES
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
extern bool safeModeFlag;

extern bool execute_P2P_traj;
extern bool execute_vel_traj;
extern bool execute_GZ_vel_traj;
extern bool policy_armed_flag;
extern bool camera_sensor_active;


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
extern float Theta_x_tr;        // [rad/s]
extern float Theta_y_tr;        // [rad/s]
extern float D_perp_tr;     // [m/s]

// CONTROLLER STATES
extern float F_thrust_flip; // [N]
extern float M_x_flip;      // [N*m]
extern float M_y_flip;      // [N*m]
extern float M_z_flip;      // [N*m]

// ==============================================
//  PARAMETER OPTIMIZATION POLICY INITIALIZATION
// ==============================================
extern float Tau_thr;   // Tau threshold
extern float G1;        // Body moment value
extern float G2;        // Deprecated gain value

// =====================================
//  SUPERVISED NN/OC_SVM INITIALIZATION
// =====================================
extern float Policy_Flip;    
extern float Policy_Action;
extern float Policy_Flip_tr;
extern float Policy_Action_tr;

typedef enum {
    PARAM_OPTIM = 0,
    SVL_POLICY = 1,
    DEEP_RL = 2,
    DEEP_RL_SB3 = 3
}PolicyType;
extern PolicyType Policy;

struct GTC_CmdPacket{
    uint8_t cmd_type; 
    float cmd_val1;
    float cmd_val2;
    float cmd_val3;
    float cmd_flag;
    bool  cmd_rx;
} __attribute__((packed));

extern struct GTC_CmdPacket GTC_Cmd;


void GTC_Command(struct GTC_CmdPacket *GTC_Cmd);
void controlOutput(const state_t *state,const sensorData_t *sensors);

#endif /* SHARED_VARIABLES_H */
