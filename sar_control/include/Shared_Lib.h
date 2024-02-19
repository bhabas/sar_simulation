#pragma once

#ifdef __cplusplus // If C++ compiler then compile accordingly
extern "C" {
#endif

#include "stabilizer_types.h"
#include "console.h"
#include "math3d.h"
#include "pm.h"
#include "quatcompress.h"
#include "nml.h"

#include "Controller_GTC.h"
#include "aideck_uart_comm.h"
#include "Traj_Funcs.h"
#include "ML_Funcs.h"
#include "Compress_States.h"

#include "ML_Params/NN_Layers_NL_DeepRL.h"



#define MOTOR_CMD_MAX 60000
#define g2Newton (9.81f/1000.0f)
#define Newton2g (1000.0f/9.81f)
#define Deg2Rad (float)M_PI/180.0f
#define Rad2Deg 180.0f/(float)M_PI



// =================================
//    INITIAL SYSTEM PARAMETERS
// =================================
extern float m;                 // [kg]
extern float Ixx;               // [kg*m^2]
extern float Iyy;               // [kg*m^2]
extern float Izz;               // [kg*m^2]
extern struct mat33 J;          // Rotational Inertia Matrix [kg*m^2]

extern float C_tf;              // Moment Coeff [Nm/N]
extern float Thrust_max;        // Max thrust per motor [g]

extern float dt;                // Controller cycle time
extern uint32_t PrevCrazyswarmTick;
extern uint32_t prev_tick;

typedef enum {
    SAR_NONE = 0,
    CRAZYFLIE = 1,
    IMPULSE_MICRO = 2,
    SO_V5 = 3,
}SAR_Types;
extern SAR_Types SAR_Type;


// =================================
//       GEOMETRIC PARAMETERS
// =================================

extern float Prop_14_x;         // Front Prop Distance - x-axis [m]
extern float Prop_14_y;         // Front Prop Distance - y-axis [m]
extern float Prop_23_x;         // Rear  Prop Distance - x-axis [m]
extern float Prop_23_y;         // Rear  Prop Distance - y-axis [m]

extern float L_eff;             // Effective Leg Length [m]
extern float Forward_Reach;     // Forward Reach [m]
extern float Collision_Radius;  // Collision Radius [m]


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
extern struct vec Kp_p;     // Pos. Proportional Gains 
extern struct vec Kd_p;     // Pos. Derivative Gains
extern struct vec Ki_p;     // Pos. Integral Gains  

extern struct vec Kp_R;     // Rot. Proportional Gains
extern struct vec Kd_R;     // Rot. Derivative Gains
extern struct vec Ki_R;     // Rot. Integral Gains


// DECLARE CTRL GAIN FLAGS
extern float kp_xf;         // Pos. Gain Flag
extern float kd_xf;         // Pos. Derivative Gain Flag


// =================================
//     BODY WRT ORIGIN STATES
// =================================
extern struct vec Pos_B_O;          // Pos [m]
extern struct vec Vel_B_O;          // Vel [m/s]
extern struct vec Accel_B_O;        // Linear Accel. [m/s^2]
extern float Accel_B_O_Mag;         // Linear Accel. Magnitude [m/s^2]

extern struct quat Quat_B_O;        // Orientation
extern struct vec Omega_B_O;        // Angular Rate [rad/s]
extern struct vec Omega_B_O_prev;   // Prev Angular Rate [rad/s^2]
extern struct vec dOmega_B_O;       // Angular Accel [rad/s^2]

extern struct mat33 R;              // Orientation as rotation matrix
extern struct vec b3;               // Current body z-axis in global coord.


// =================================
//     BODY WRT PLANE STATES
// =================================
extern struct vec Pos_P_B;         // Pos [m]
extern struct vec Vel_B_P;         // Vel [m/s]
extern struct quat Quat_P_B;       // Orientation
extern struct vec Omega_B_P;       // Angular Rate [rad/s]

// RELATIVE STATES
extern float D_perp;               // Distance from body to plane [m]
extern float D_perp_CR;            // Distance from CR to plane [m]
extern float Vel_mag_B_P;          // Velocity magnitude relative [m/s]
extern float Vel_angle_B_P;        // Velocity angle relative [deg]


// =================================
//         DESIRED STATES
// =================================
extern struct vec x_d;          // Pos-desired [m]
extern struct vec v_d;          // Vel-desired [m/s]
extern struct vec a_d;          // Acc-desired [m/s^2]

extern struct vec b1_d;         // Desired body x-axis in global coord. 
extern struct vec b2_d;         // Desired body y-axis in global coord.
extern struct vec b3_d;         // Desired body z-axis in global coord.
extern struct mat33 R_d;        // Desired rotational matrix from b_d vectors

extern struct quat quat_d;      // Orientation-desired [qx,qy,qz,qw]
extern struct vec omega_d;      // Omega-desired [rad/s]
extern struct vec domega_d;     // Ang. Acc-desired [rad/s^2]

// =================================
//         STATE ERRORS
// =================================
extern struct vec e_x;  // Pos-error [m]
extern struct vec e_v;  // Vel-error [m/s]
extern struct vec e_PI; // Pos. Integral-error [m*s]

extern struct vec e_R;  // Rotation-error [rad]
extern struct vec e_w;  // Omega-error [rad/s]
extern struct vec e_RI; // Rot. Integral-error [rad*s]


// =================================
//       CONTROLLER ACTUATIONS
// =================================
extern struct vec F_thrust_ideal;   // Ideal thrust vector [N]
extern float F_thrust;              // Implemented body thrust [N]
extern struct vec M;                // Implemented body moments [N*m]
extern struct vec M_d;              // Desired body moment [N*m]

// MOTOR THRUST ACTIONS
extern float f_thrust_g;        // Motor thrust - Thrust [g]
extern float f_roll_g;          // Motor thrust - Roll   [g]
extern float f_pitch_g;         // Motor thrust - Pitch  [g]
extern float f_yaw_g;           // Motor thrust - Yaw    [g]

// MOTOR THRUSTS
extern float M1_thrust;
extern float M2_thrust;
extern float M3_thrust;
extern float M4_thrust;

// MOTOR M_CMD VALUES
extern uint16_t M1_CMD; 
extern uint16_t M2_CMD; 
extern uint16_t M3_CMD; 
extern uint16_t M4_CMD; 

// CONTROL OVERRIDE VALUES
extern uint16_t M_CMD_override[4];    // Motor M_CMD values
extern float thrust_override[4];    // Motor thrusts [g] 


// =================================
//        OPTICAL FLOW STATES
// =================================

// OPTICAL FLOW STATES (GROUND TRUTH)
extern float Tau;           // [s]
extern float Tau_CR;        // [s]
extern float Theta_x;       // [rad/s] 
extern float Theta_y;       // [rad/s]

// OPTICAL FLOW STATES (CAMERA ESTIMATE)
extern float Tau_Cam;      // [s]
extern float Theta_x_Cam;  // [rad/s]
extern float Theta_y_Cam;  // [rad/s]

// CAMERA PARAMETERS
extern float IW;            // Image Width [m]
extern float IH;            // Image Height [m]
extern float focal_len;     // Focal Length [m]
extern int32_t N_up;        // Pixel Count Horizontal [m]
extern int32_t N_vp;        // Pixel Count Vertical [m]
extern int32_t Cam_dt;      // Time Between Images [ms]

extern int32_t UART_arr[UART_ARR_SIZE];
extern bool isOFUpdated;

// =================================
//  FLAGS AND SYSTEM INITIALIZATION
// =================================

// CONTROLLER FLAGS
extern bool Tumbled_Flag;
extern bool TumbleDetect_Flag;
extern bool MotorStop_Flag;
extern bool AngAccel_Flag;
extern bool Armed_Flag;
extern bool CustomThrust_Flag;
extern bool CustomMotorCMD_Flag;

// SENSOR FLAGS
extern bool CamActive_Flag;

// =================================
//       POLICY INITIALIZATION
// =================================

// POLICY SETTING
typedef enum {
    PARAM_OPTIM = 0,
    DEEP_RL_SB3 = 1,
    DEEP_RL_ONBOARD = 2,
}PolicyType;
extern PolicyType Policy;

extern nml_mat* X_input;    // STATE MATRIX TO BE INPUT INTO POLICY
extern nml_mat* Y_output;   // POLICY OUTPUT MATRIX

// POLICY FLAGS
extern bool Policy_Armed_Flag;
extern bool Trg_Flag;
extern bool onceFlag;

// POLICY TRIGGER/ACTION VALUES
extern float Policy_Trg_Action;  
extern float Policy_Rot_Action;

extern float ACTION_MIN;
extern float ACTION_MAX;


// ===============================
//  DEEP RL POLICY INITIALIZATION
// ===============================

extern NN NN_DeepRL;
extern NN NN_DeepRL2;
extern float Policy_Rot_threshold;




// ==========================================
//  RECORD SYSTEM STATES AT POLICY TRIGGER
// ==========================================

// BODY WRT ORIGIN STATES
extern struct vec Pos_B_O_trg;     // Pos [m]
extern struct vec Vel_B_O_trg;     // Vel [m/s]
extern struct quat Quat_B_O_trg;   // Orientation
extern struct vec Omega_B_O_trg;   // Angular Rate [rad/s]

// BODY WRT PLANE STATES
extern struct vec Pos_P_B_trg;              // Pos [m]
extern struct vec Vel_B_P_trg;              // Vel [m/s]
extern struct quat Quat_P_B_trg;            // Orientation
extern struct vec Omega_B_P_trg;            // Angular Rate [rad/s]

// RELATIVE STATES
extern float D_perp_trg;                // Distance perp to plane [m]
extern float D_perp_CR_trg;             // Distance from CR to plane [m]
extern float Vel_mag_B_P_trg;           // Velocity magnitude relative [m/s]
extern float Vel_angle_B_P_trg;         // Velocity angle relative [deg]



// OPTICAL FLOW STATES
extern float Tau_trg;                   // [rad/s]
extern float Tau_CR_trg;                // [rad/s]
extern float Theta_x_trg;               // [rad/s]
extern float Theta_y_trg;               // [rad/s]

// OPTICAL FLOW CAMERA ESTIMATES
extern float Tau_Cam_trg;               // [rad/s]
extern float Theta_x_Cam_trg;           // [rad/s]
extern float Theta_y_Cam_trg;           // [rad/s]

// POLICY TRIGGER/ACTION VALUES
extern float Policy_Trg_Action_trg;    
extern float Policy_Rot_Action_trg;

// =================================
//  RECORD SYSTEM STATES AT IMPACT
// =================================
extern bool Impact_Flag_OB;
extern float Accel_B_O_Mag_impact_OB;      // Linear Accel. Magnitude [m/s^2]
extern struct vec Pos_B_O_impact_OB;       // Pos [m]
extern struct quat Quat_B_O_impact_OB;     // Orientation

extern struct vec Vel_B_P_impact_OB;       // Vel [m/s]
extern struct vec Omega_B_P_impact_OB;     // Angular Rate [rad/s]

// =================================
//    LANDING SURFACE PARAMETERS
// =================================
extern float Plane_Angle_deg;   // Plane Angle [deg]
extern struct vec r_P_O;        // Plane Position Vector        [m]


// =================================
//         ROTATION MATRICES
// =================================
extern struct mat33 R_WP;       // Rotation matrix from world to plane
extern struct mat33 R_PW;       // Rotation matrix from plane to world




// CTRL COMMAND PACKETS
struct CTRL_CmdPacket{
    uint8_t cmd_type; 
    float cmd_val1;
    float cmd_val2;
    float cmd_val3;
    float cmd_flag;
    bool  cmd_rx;
} __attribute__((packed));
extern struct CTRL_CmdPacket CTRL_Cmd;





void CTRL_Command(struct CTRL_CmdPacket *CTRL_Cmd);
void controlOutput(const state_t *state, const sensorData_t *sensors);
uint16_t thrust2Motor_CMD(float f);
void updateRotationMatrices();
bool updateOpticalFlowEst();
bool updateOpticalFlowAnalytic(const state_t *state, const sensorData_t *sensors);
float firstOrderFilter(float newValue, float prevValue, float alpha);






// =================================
//    ADDITIONAL MATH3D FUNCTIONS
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

