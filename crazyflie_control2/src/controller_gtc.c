// CF HEADERS
#include "controller_gtc.h"
#include "NN_funcs.h"

// =================================
//    CONTROL GAIN INITIALIZATION
// =================================
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


// INIT CTRL GAIN VECTORS 
struct vec Kp_p; // Pos. Proportional Gains 
struct vec Kd_p; // Pos. Derivative Gains
struct vec Ki_p; // Pos. Integral Gains  

struct vec Kp_R; // Rot. Proportional Gains
struct vec Kd_R; // Rot. Derivative Gains
struct vec Ki_R; // Rot. Integral Gains

// CONTROLLER GAIN FLAGS
float kp_xf = 1; // Pos. Gain Flag
float kd_xf = 1; // Pos. Derivative Gain Flag


// SYSTEM PARAMETERS
float m = 0.0376;           // [kg]
float g = 9.81f;            // [m/s^2]
float h_ceiling = 2.10f;    // [m]
struct mat33 J = {          // Rotational Inertia Matrix [kg*m^2]
    .m[0][0]=1.65717e-5f, 
    .m[1][1]=1.65717e-5f, 
    .m[2][2]=2.92617e-5f
}; 
static float dt = (float)(1.0f/RATE_500_HZ);


// INIT STATE VALUES
struct vec statePos = {0.0,0.0f,0.0f};         // Pos [m]
struct vec stateVel = {0.0f,0.0f,0.0f};         // Vel [m/s]
struct quat stateQuat = {0.0f,0.0f,0.0f,1.0f};  // Orientation
struct vec stateOmega = {0.0f,0.0f,0.0f};       // Angular Rate [rad/s]

// OPTICAL FLOW STATES
float Tau = 0.0f;   // [s]
float OFx = 0.0f;  // [rad/s]
float OFy = 0.0f;  // [rad/s] 
float RREV = 0.0f;  // [rad/s]
float d_ceil = 0.0f;

static struct mat33 R; // Orientation as rotation matrix
static struct vec stateEul = {0.0f,0.0f,0.0f}; // Pose in Euler Angles [YZX Notation]


// INIT DESIRED STATES
struct vec x_d = {0.0f,0.0f,0.4f}; // Pos-desired [m]
struct vec v_d = {0.0f,0.0f,0.0f}; // Vel-desired [m/s]
struct vec a_d = {0.0f,0.0f,0.0f}; // Acc-desired [m/s^2]

struct quat quat_d = {0.0f,0.0f,0.0f,1.0f}; // Orientation-desired [qx,qy,qz,qw]
struct vec eul_d = {0.0f,0.0f,0.0f};        // Euler Angle-desired [rad? deg? TBD]
struct vec omega_d = {0.0f,0.0f,0.0f};      // Omega-desired [rad/s]
struct vec domega_d = {0.0f,0.0f,0.0f};     // Ang. Acc-desired [rad/s^2]

struct vec M_d = {0.0f,0.0f,0.0f};   // Desired moment [N*m]

static struct vec b1_d = {1.0f,0.0f,0.0f};    // Desired body x-axis in global coord. [x,y,z]
static struct vec b2_d;    // Desired body y-axis in global coord.
static struct vec b3_d;    // Desired body z-axis in global coord.
static struct vec b3;      // Current body z-axis in global coord.

static struct mat33 R_d;   // Desired rotational matrix from b_d vectors
static struct vec e_3 = {0.0f, 0.0f, 1.0f}; // Global z-axis

// STATE ERRORS
struct vec e_x;  // Pos-error [m]
struct vec e_v;  // Vel-error [m/s]
struct vec e_PI; // Pos. Integral-error [m*s]

struct vec e_R;  // Rotation-error [rad]
struct vec e_w;  // Omega-error [rad/s]
struct vec e_RI; // Rot. Integral-error [rad*s]

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
struct vec F_thrust_ideal;           // Ideal thrust vector [N]
float F_thrust = 0.0f;               // Implemented body thrust [N]
struct vec M = {0.0f,0.0f,0.0f};     // Implemented body moments [N*m]


// MOTOR THRUSTS
float f_thrust_g = 0.0f; // Motor thrust - Thrust [g]
float f_roll_g = 0.0f;   // Motor thrust - Roll   [g]
float f_pitch_g = 0.0f;  // Motor thrust - Pitch  [g]
float f_yaw_g = 0.0f;    // Motor thrust - Yaw    [g]


// MOTOR VARIABLES
uint16_t M1_pwm = 0; 
uint16_t M2_pwm = 0; 
uint16_t M3_pwm = 0; 
uint16_t M4_pwm = 0; 



// =================================
//  FLAGS AND SYSTEM INITIALIZATION
// =================================

// CONTROLLER FLAGS
bool tumbled = false;
bool tumble_detection = true;
bool motorstop_flag = false;
bool errorReset = false; // Resets error vectors (removed integral windup)

bool execute_traj = false;
bool policy_armed_flag = false;

bool flip_flag = false;
bool onceFlag = false;

bool Moment_flag = false;
bool attCtrlEnable = false;


// DEFINE POLICY TYPE ACTIVATED
typedef enum {
    RL = 0, // Reinforcement Learning
    NN = 1  // Neural Network
} Policy_Type;
Policy_Type POLICY_TYPE = RL; // Default to RL

// ======================================
//  RECORD SYSTEM STATES AT FLIP TRIGGER
// ======================================

// CARTESIAN STATES
struct vec statePos_tr = {0.0f,0.0f,0.0f};         // Pos [m]
struct vec stateVel_tr = {0.0f,0.0f,0.0f};         // Vel [m/s]
struct quat stateQuat_tr = {0.0f,0.0f,0.0f,1.0f};  // Orientation
struct vec stateOmega_tr = {0.0f,0.0f,0.0f};       // Angular Rate [rad/s]

// OPTICAL FLOW STATES
float Tau_tr = 0.0f;    // [rad/s]
float OFx_tr = 0.0f;    // [rad/s]
float OFy_tr = 0.0f;    // [rad/s]
float RREV_tr = 0.0f;   // [rad/s]
float d_ceil_tr = 0.0f; // [m/s]

// CONTROLLER STATES
float F_thrust_flip = 0.0f; // [N]
float M_x_flip = 0.0f;      // [N*m]
float M_y_flip = 0.0f;      // [N*m]
float M_z_flip = 0.0f;      // [N*m]

// ====================================
//  CONSTANT VEL TRAJECTORY GENERATION
// ====================================
typedef enum {
    x = 0, 
    y = 1,
    z = 2
} axis_direction;
axis_direction traj_type;

static struct vec s_0_t = {0.0f, 0.0f, 0.0f};
static struct vec v_t = {0.0f, 0.0f, 0.0f};
static struct vec a_t = {0.0f, 0.0f, 0.0f};
static struct vec T = {0.0f, 0.0f, 0.0f};
static struct vec t_traj = {0.0f, 0.0f, 0.0f};

// ==========================
//  RL POLICY INITIALIZATION
// ==========================
float RREV_thr = 0.0f;  // RREV trigger
float G1 = 0.0f;        // Body moment value
float G2 = 0.0f;        // Deprecated state value


// ===============================
//  NEURAL NETWORK INITIALIZATION
// ===============================
static nml_mat* X;  // STATE MATRIX TO BE INPUT INTO NN

// NN INPUT SCALERS
static Scaler Scaler_Flip;      // Scale input vector for NN
static Scaler Scaler_Policy;

// NN WEIGHTS
static nml_mat* W_flip[3];  
static nml_mat* W_policy[3];

// NN BIASES
static nml_mat* b_flip[3];  
static nml_mat* b_policy[3];

// NN OUTPUTS
float NN_flip = 0.0f;           // NN output value for flip classification
float NN_policy = 0.0f;         // NN output value for policy My

// NN OUTPUTS AT FLIP TRIGGER
float NN_tr_flip = 0.0f;        // NN value at flip trigger
float NN_tr_policy = 0.0f;      // NN policy value at flip trigger

void controllerGTCInit(void)
{
    controllerGTCTest();
    X = nml_mat_new(3,1);
    initNN_Layers(&Scaler_Flip,W_flip,b_flip,NN_Params_Flip,3);
    initNN_Layers(&Scaler_Policy,W_policy,b_policy,NN_Params_Policy,3);
    controllerGTCReset();
    consolePrintf("GTC Initiated\n");
}

void controllerGTCReset(void)
{
    consolePrintf("GTC Reset\n");

}

bool controllerGTCTest(void)
{
    return true;
}

void GTC_Command(setpoint_t *setpoint)
{   
    
    
}

void controllerGTCTraj()
{
   

    
}


int val = 0;
void controllerGTC(control_t *control, setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick)
{
    if (RATE_DO_EXECUTE(5, tick)) {

        // SYSTEM PARAMETERS 
        J = mdiag(1.65717e-5f, 1.66556e-5f, 2.92617e-5f); // Rotational Inertia of CF [kg m^2]

        // CONTROL GAINS
        Kp_p = mkvec(P_kp_xy,P_kp_xy,P_kp_z);
        Kd_p = mkvec(P_kd_xy,P_kd_xy,P_kd_z);
        Ki_p = mkvec(P_ki_xy,P_ki_xy,P_ki_z);

        Kp_R = mkvec(R_kp_xy,R_kp_xy,R_kp_z);
        Kd_R = mkvec(R_kd_xy,R_kd_xy,R_kd_z);
        Ki_R = mkvec(R_ki_xy,R_ki_xy,R_ki_z);

        // =========== STATE DEFINITIONS =========== //
        statePos = mkvec(state->position.x, state->position.y, state->position.z);                      // [m]
        stateVel = mkvec(state->velocity.x, state->velocity.y, state->velocity.z);                      // [m]
        stateOmega = mkvec(radians(sensors->gyro.x), radians(sensors->gyro.y), radians(sensors->gyro.z));   // [rad/s]
        stateQuat = mkquat(state->attitudeQuaternion.x,
                        state->attitudeQuaternion.y,
                        state->attitudeQuaternion.z,
                        state->attitudeQuaternion.w);

        // EULER ANGLES EXPRESSED IN YZX NOTATION
        stateEul = quat2eul(stateQuat);
        stateEul.x = degrees(stateEul.x);
        stateEul.y = degrees(stateEul.y);
        stateEul.z = degrees(stateEul.z);

        RREV = sensors->RREV;
        OFx = sensors->OFx;
        OFy = sensors->OFy;
        d_ceil = h_ceiling - statePos.x;

        X->data[0][0] = RREV;
        X->data[1][0] = OFy;
        X->data[2][0] = d_ceil; // d_ceiling [m]
    }

}
