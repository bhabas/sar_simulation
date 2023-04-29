#pragma once

#ifdef __cplusplus // If C++ compiler then compile accordingly
extern "C" {
#endif

#include "stabilizer_types.h"
#include "console.h"
#include "math3d.h"
#include "pm.h"
#include "quatcompress.h"

#include "controller_GTC.h"
#include "traj_funcs.h"
#include "CompressStates.h"


#define PWM_MAX 60000
#define g2Newton (9.81f/1000.0f)
#define Newton2g (1000.0f/9.81f)
#define Deg2Rad (float)M_PI/180.0f
#define Rad2Deg 180.0f/(float)M_PI



// DECLARE SYSTEM PARAMETERS
extern float m;         // [kg]
extern float Ixx;       // [kg*m^2]
extern float Iyy;       // [kg*m^2]
extern float Izz;       // [kg*m^2]
extern struct mat33 J;  // Rotational Inertia Matrix [kg*m^2]

extern float Prop_Dist;    // COM to Prop along x-axis [m]
extern float C_tf;  // Moment Coeff [Nm/N]
extern float f_max; // Max thrust per motor [g]

extern float dt;    // Controller cycle time




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

// STATE ERRORS
extern struct vec e_x;  // Pos-error [m]
extern struct vec e_v;  // Vel-error [m/s]
extern struct vec e_PI; // Pos. Integral-error [m*s]

extern struct vec e_R;  // Rotation-error [rad]
extern struct vec e_w;  // Omega-error [rad/s]
extern struct vec e_RI; // Rot. Integral-error [rad*s]


// DECLARE CONTROLLER ACTUATIONS
extern struct vec F_thrust_ideal;   // Ideal thrust vector [N]
extern float F_thrust;              // Implemented body thrust [N]
extern struct vec M;                // Implemented body moments [N*m]
extern struct vec M_d;              // Desired body moment [N*m]

// DECLARE MOTOR THRUST ACTIONS
extern float f_thrust_g;        // Motor thrust - Thrust [g]
extern float f_roll_g;          // Motor thrust - Roll   [g]
extern float f_pitch_g;         // Motor thrust - Pitch  [g]
extern float f_yaw_g;           // Motor thrust - Yaw    [g]

// DECLARE MOTOR THRUSTS
extern float M1_thrust;
extern float M2_thrust;
extern float M3_thrust;
extern float M4_thrust;

// MOTOR PWM VALUES
extern uint16_t M1_pwm; 
extern uint16_t M2_pwm; 
extern uint16_t M3_pwm; 
extern uint16_t M4_pwm; 


// CONTROL OVERRIDE VALUES
extern uint16_t PWM_override[4];    // Motor PWM values
extern float thrust_override[4];    // Motor thrusts [g] 


// =================================
//          SENSORY VALUES
// =================================

// OPTICAL FLOW STATES
extern float Tau;           // [s]
extern float Theta_x;       // [rad/s] 
extern float Theta_y;       // [rad/s]
extern float Theta_z;       // [rad/s]

// ANALYTICAL OPTICAL FLOW STATES
extern float Tau_calc;      // [s]
extern float Theta_x_calc;  // [rad/s] 
extern float Theta_y_calc;  // [rad/s]
extern float D_perp_calc;   // [m]

// ESTIMATED OPTICAL FLOW STATES
extern float Tau_est;      // [s]
extern float Theta_x_est;  // [rad/s]
extern float Theta_y_est;  // [rad/s]
extern float D_perp_est;   // [m]


// =================================
//  FLAGS AND SYSTEM INITIALIZATION
// =================================

// CONTROLLER FLAGS
extern bool tumbled;
extern bool tumble_detection;
extern bool motorstop_flag;
extern bool moment_flag;
extern bool attCtrlEnable;
extern bool safeModeEnable;
extern bool customThrust_flag;
extern bool customPWM_flag;

// POLICY FLAGS
extern bool policy_armed_flag;
extern bool flip_flag;
extern bool onceFlag;

// SENSOR FLAGS
extern bool camera_sensor_active;

// =================================
//       POLICY INITIALIZATION
// =================================

// POLICY SETTING
typedef enum {
    PARAM_OPTIM = 0,
    SVL_POLICY = 1,
    DEEP_RL = 2,
    DEEP_RL_SB3 = 3
}PolicyType;
extern PolicyType Policy;

// POLICY FLAGS
extern bool policy_armed_flag;
extern bool flip_flag;
extern bool onceFlag;

// POLICY TRIGGER/ACTION VALUES
extern float Policy_Flip;  
extern float Policy_Action;



// ======================================
//  RECORD SYSTEM STATES AT FLIP TRIGGER
// ======================================

// CARTESIAN STATES
extern struct vec statePos_tr;      // Pos [m]
extern struct vec stateVel_tr;      // Vel [m/s]
extern struct quat stateQuat_tr;    // Orientation
extern struct vec stateOmega_tr;    // Angular Rate [rad/s]

// OPTICAL FLOW STATES
extern float Tau_tr;                // [rad/s]
extern float Theta_x_tr;            // [rad/s]
extern float Theta_y_tr;            // [rad/s]
extern float D_perp_tr;             // [m/s]

// CONTROLLER STATES
extern float F_thrust_flip;         // [N]
extern float M_x_flip;              // [N*m]
extern float M_y_flip;              // [N*m]
extern float M_z_flip;              // [N*m]

// POLICY TRIGGER/ACTION VALUES
extern float Policy_Flip_tr;    
extern float Policy_Action_tr;

// =================================
//    LANDING SURFACE PARAMETERS
// =================================

// LANDING SURFACE PARAMETERS
extern float Plane_Angle;
extern struct vec t_x;          // Plane Unit Tangent Vector
extern struct vec t_y;          // Plane Unit Tangent Vector
extern struct vec n_hat;        // Plane Unit Normal Vector

extern struct vec r_PO;         // Plane Position Vector        [m]
extern struct vec r_BO;         // Quad Position Vector         [m]
extern struct vec r_PB;         // Quad-Plane Distance Vector   [m]
extern struct vec V_BO;         // Quad Velocity Vector         [m/s]

// DECLARE RELATIVE STATES
extern float D_perp;            // Distance perp to plane [m]
extern float V_perp;            // Velocity perp to plane [m/s]
extern float V_tx;              // Tangent_x velocity [m/s]
extern float V_ty;              // Tangent_y velocity [m/s]



// GTC COMMAND PACKETS
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
void controlOutput(const state_t *state, const sensorData_t *sensors);
uint16_t thrust2PWM(float f);
void calcPlaneNormal(float Plane_Angle);






// =================================
//    ADDITIONAL MATH FUNCTIONS
// =================================

// Construct a matrix A from vector v such that Ax = cross(v, x)
static inline struct mat33 hat(struct vec v) {
	struct mat33 m;
	m.m[0][0] = 0;
	m.m[0][1] = -v.z;
	m.m[0][2] = v.y;
	m.m[1][0] = v.z;
	m.m[1][1] = 0;
	m.m[1][2] = -v.x;
	m.m[2][0] = -v.y;
	m.m[2][1] = v.x;
	m.m[2][2] = 0;
	return m;
}

// Construct a vector v from matrix A such that Ax = cross(v, x)
static inline struct vec dehat(struct mat33 m) {
	struct vec v;

	v.x = m.m[2][1];
	v.y = m.m[0][2];
	v.z = m.m[1][0];
	
	return v;
}

// Convert quaternion to (roll, pitch, yaw) Euler angles using Tait-Bryan convention [YZX]
//  - Pitch, then yaw about new pitch axis, then roll about new roll axis
//  - Notation allows greater than 90 deg pitch and roll angles
static inline struct vec quat2eul(struct quat q) {
	// from https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles

	struct vec eul;
	float R11,R21,R31,R22,R23;


	// CALC NEEDED ROTATION MATRIX COMPONENTS FROM QUATERNION
    R11 = 1.0f - 2.0f*( fsqr(q.y) + fsqr(q.z) );
    R21 = 2.0f*(q.x*q.y + q.z*q.w);
    R31 = 2.0f*(q.x*q.z - q.y*q.w);

    R22 = 1.0f - 2.0f*( fsqr(q.x) + fsqr(q.z) );
    R23 = 2.0f*(q.y*q.z - q.x*q.w);


	// CONVERT ROTATION MATRIX COMPONENTS TO EULER ANGLES (YZX NOTATION)
	eul.x = atan2f(-R23,R22); 	// Roll
	eul.y = atan2f(-R31,R11); 	// Pitch
	eul.z = asinf(R21); 		// Yaw

	return eul;
}

static inline void printvec(struct vec v){
	consolePrintf("%.4f, %.4f, %.4f\n", (double)v.x, (double)v.y, (double)v.z);
	return;
}

static inline void printquat(struct quat q){
	consolePrintf("%.4f, %.4f, %.4f %.4f\n", (double)q.x, (double)q.y, (double)q.z, (double)q.w);
	return;
}

static inline void printmat(struct mat33 m){
    struct vec vrow_0 = mrow(m,0);
    struct vec vrow_1 = mrow(m,1);
    struct vec vrow_2 = mrow(m,2);

    printvec(vrow_0);
    printvec(vrow_1);
    printvec(vrow_2);
	consolePrintf("\n");

	return;
}

#ifdef __cplusplus
}
#endif

