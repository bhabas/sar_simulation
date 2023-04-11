#pragma once

#ifdef __cplusplus // If C++ compiler then compile accordingly
extern "C" {
#endif

#include "stabilizer_types.h"
#include "console.h"
#include "controller_GTC.h"
#include "math3d.h"

struct GTC_CmdPacket{
    uint8_t cmd_type; 
    float cmd_val1;
    float cmd_val2;
    float cmd_val3;
    float cmd_flag;
    bool  cmd_rx;
} __attribute__((packed));

extern struct GTC_CmdPacket GTC_Cmd;

// DECLARE SYSTEM PARAMETERS
extern float m;     // [kg]
extern float Ixx;   // [kg*m^2]
extern float Iyy;   // [kg*m^2]
extern float Izz;   // [kg*m^2]
extern float dt;    // Controller cycle time

extern float dp;    // COM to Prop along x-axis [m]
extern float c_tf;  // Moment Coeff [Nm/N]

// =================================
//    CONTROL GAIN DECLARATIONS
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


// DECLARE CTRL GAIN VECTORS 
extern struct vec Kp_p; // Pos. Proportional Gains 
extern struct vec Kd_p; // Pos. Derivative Gains
extern struct vec Ki_p; // Pos. Integral Gains  

extern struct vec Kp_R; // Rot. Proportional Gains
extern struct vec Kd_R; // Rot. Derivative Gains
extern struct vec Ki_R; // Rot. Integral Gains


// DECLARE CTRL GAIN FLAGS
extern float kp_xf; // Pos. Gain Flag
extern float kd_xf; // Pos. Derivative Gain Flag


// DECLARE STATE VALUES
extern struct vec statePos;     // Pos [m]
extern struct vec stateVel;     // Vel [m/s]
extern struct quat stateQuat;   // Orientation
extern struct vec stateEul;     // Orientation in Euler Angles [YZX Notation]
extern struct vec stateOmega;   // Angular Rate [rad/s]

extern struct mat33 R;          // Orientation as rotation matrix
extern struct vec b3;           // Current body z-axis in global coord.


// DECLARE DESIRED STATES
extern struct vec x_d;          // Pos-desired [m]
extern struct vec v_d;          // Vel-desired [m/s]
extern struct vec a_d;          // Acc-desired [m/s^2]

extern struct quat quat_d;      // Orientation-desired [qx,qy,qz,qw]
extern struct vec eul_d;        // Euler Angle-desired [deg]

extern struct vec b1_d;         // Desired body x-axis in global coord. 
extern struct vec b2_d;         // Desired body y-axis in global coord.
extern struct vec b3_d;         // Desired body z-axis in global coord.
extern struct mat33 R_d;        // Desired rotational matrix from b_d vectors


extern struct vec omega_d;      // Omega-desired [rad/s]
extern struct vec domega_d;     // Ang. Acc-desired [rad/s^2]


// DECLARE CONTROLLER ACTUATIONS
extern struct vec F_thrust_ideal;   // Ideal thrust vector [N]
extern float F_thrust;              // Implemented body thrust [N]
extern struct vec M;                // Implemented body moments [N*m]

// DECLARE MOTOR THRUST ACTIONS
extern float f_thrust_g;        // Motor thrust - Thrust [g]
extern float f_roll_g;          // Motor thrust - Roll   [g]
extern float f_pitch_g;         // Motor thrust - Pitch  [g]
extern float f_yaw_g;           // Motor thrust - Yaw    [g]



void GTC_Command(struct GTC_CmdPacket *GTC_Cmd);
void controlOutput(const state_t *state, const sensorData_t *sensors);

#ifdef __cplusplus
}
#endif

