// CF HEADERS
#include "controller_gtc.h"
#include "ML_funcs.h"
#include "CompressedStates.h"

// =================================
//    CONTROL GAIN INITIALIZATION
// =================================
// (INITIAL VALUES THAT ARE OVERWRITTEN BY Ctrl_Gains.yaml)

// XY POSITION PID
float P_kp_xy = 0.5f;
float P_kd_xy = 0.3f;
float P_ki_xy = 0.0f;
float i_range_xy = 0.3f;

// Z POSITION PID
float P_kp_z = 1.20f;
float P_kd_z = 0.35f;
float P_ki_z = 0.0f;
float i_range_z = 0.25f;

// XY ATTITUDE PID
float R_kp_xy = 0.02f;
float R_kd_xy = 0.008f;
float R_ki_xy = 0.0f;
float i_range_R_xy = 1.0f;

// Z ATTITUDE PID
float R_kp_z = 0.003f;
float R_kd_z = 0.001f;
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


// INITIAL SYSTEM PARAMETERS
float m = 34.3e-3f;         // [kg]
float Ixx = 15.83e-6f;      // [kg*m^2]
float Iyy = 17.00e-6f;      // [kg*m^2]
float Izz = 31.19e-6f;      // [kg*m^2]

float g = 9.81f;        // [m/s^2]
static struct mat33 J; // Rotational Inertia Matrix [kg*m^2]


static float dp = 0.0325; // COM to Prop along x-axis [m]
static float const kf = 2.2e-8f;    // Thrust Coeff [N/(rad/s)^2]
static float const c_tf = 0.00618f; // Moment Coeff [Nm/N]
static float dt = (float)(1.0f/RATE_500_HZ);


// INIT STATE VALUES
struct vec statePos = {0.0,0.0f,0.0f};          // Pos [m]
struct vec stateVel = {0.0f,0.0f,0.0f};         // Vel [m/s]
struct quat stateQuat = {0.0f,0.0f,0.0f,1.0f};  // Orientation
struct vec stateOmega = {0.0f,0.0f,0.0f};       // Angular Rate [rad/s]
float V_mag = 0.0f;

// OPTICAL FLOW STATES
float Tau = 0.0f;       // [s]
float Theta_x = 0.0f;   // [rad/s] 
float Theta_y = 0.0f;   // [rad/s]
float D_perp = 0.0f;    // [m]

// ESTIMATED OPTICAL FLOW STATES
float Tau_est = 0.0f;       // [s]
float Theta_x_est = 0.0f;   // [rad/s]
float Theta_y_est = 0.0f;   // [rad/s]


static struct mat33 R; // Orientation as rotation matrix
struct vec stateEul = {0.0f,0.0f,0.0f}; // Pose in Euler Angles [YZX Notation]


// INIT DESIRED STATES
struct vec x_d = {0.0f,0.0f,0.4f};      // Pos-desired [m]
struct vec v_d = {0.0f,0.0f,0.0f};      // Vel-desired [m/s]
struct vec a_d = {0.0f,0.0f,0.0f};      // Acc-desired [m/s^2]
float yaw_d = 0.0f;
struct vec b1_d = {1.0f,0.0f,0.0f};     // Desired body x-axis in global coord. [x,y,z]

struct quat quat_d = {0.0f,0.0f,0.0f,1.0f}; // Orientation-desired [qx,qy,qz,qw]
struct vec eul_d = {0.0f,0.0f,0.0f};        // Euler Angle-desired [rad? deg? TBD]
struct vec omega_d = {0.0f,0.0f,0.0f};      // Omega-desired [rad/s]
struct vec domega_d = {0.0f,0.0f,0.0f};     // Ang. Acc-desired [rad/s^2]

struct vec M_d = {0.0f,0.0f,0.0f};   // Desired moment [N*m]

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

// MOTOR THRUST ACTIONS
float f_thrust_g = 0.0f; // Motor thrust - Thrust [g]
float f_roll_g = 0.0f;   // Motor thrust - Roll   [g]
float f_pitch_g = 0.0f;  // Motor thrust - Pitch  [g]
float f_yaw_g = 0.0f;    // Motor thrust - Yaw    [g]

// INDIVIDUAL MOTOR THRUSTS
float M1_thrust = 0.0f;
float M2_thrust = 0.0f;
float M3_thrust = 0.0f;
float M4_thrust = 0.0f;


// MOTOR VARIABLES
uint16_t M1_pwm = 0; 
uint16_t M2_pwm = 0; 
uint16_t M3_pwm = 0; 
uint16_t M4_pwm = 0; 

// CONTROL OVERRIDE VALUES

uint16_t PWM_override[4] = {0,0,0,0};               // Motor PWM values
float thrust_override[4] = {0.0f,0.0f,0.0f,0.0f}; // Motor thrusts [g] 



// =================================
//  FLAGS AND SYSTEM INITIALIZATION
// =================================

// CONTROLLER FLAGS
bool tumbled = false;
bool tumble_detection = true;
bool motorstop_flag = false;
bool safeModeFlag = false;

bool execute_P2P_traj = false;
bool execute_vel_traj = false;
bool execute_GZ_vel_traj = false;
bool policy_armed_flag = false;
bool camera_sensor_active = false;

bool flip_flag = false;
bool onceFlag = false;

bool moment_flag = false;
bool attCtrlEnable = false;
bool safeModeEnable = true;
bool customThrust_flag = false;
bool customPWM_flag = false;


// DEFINE POLICY TYPE ACTIVATED
PolicyType Policy = PARAM_OPTIM;


// ======================================
//  RECORD SYSTEM STATES AT FLIP TRIGGER
// ======================================

// CARTESIAN STATES
struct vec statePos_tr = {0.0f,0.0f,0.0f};         // Pos [m]
struct vec stateVel_tr = {0.0f,0.0f,0.0f};         // Vel [m/s]
struct quat stateQuat_tr = {0.0f,0.0f,0.0f,1.0f};  // Orientation
struct vec stateOmega_tr = {0.0f,0.0f,0.0f};       // Angular Rate [rad/s]

// OPTICAL FLOW STATES
float Tau_tr = 0.0f;        // [rad/s]
float Theta_x_tr = 0.0f;    // [rad/s]
float Theta_y_tr = 0.0f;    // [rad/s]
float D_perp_tr = 0.0f;     // [m/s]

// CONTROLLER STATES
float F_thrust_flip = 0.0f; // [N]
float M_x_flip = 0.0f;      // [N*m]
float M_y_flip = 0.0f;      // [N*m]
float M_z_flip = 0.0f;      // [N*m]

// ====================================
//  CONSTANT VEL TRAJECTORY GENERATION
// ====================================
typedef enum {
    x_axis = 0, 
    y_axis = 1,
    z_axis = 2
} axis_direction;
axis_direction traj_type;

static struct vec P2P_traj_flag = {0.0f, 0.0f, 0.0f};
static struct vec s_0_t = {0.0f, 0.0f, 0.0f};   // Traj Start Point [m]
static struct vec s_f_t = {0.0f, 0.0f, 0.0f};   // Traj End Point [m]
static struct vec v_t = {0.0f, 0.0f, 0.0f};     // Traj Vel [m/s]
static struct vec a_t = {0.0f, 0.0f, 0.0f};     // Traj Accel [m/s^2]
static struct vec T = {0.0f, 0.0f, 0.0f};       // Traj completion time [s]
static struct vec t_traj = {0.0f, 0.0f, 0.0f};  // Traj time counter [s]

// ==========================
//  RL POLICY INITIALIZATION
// ==========================
float Tau_thr = 0.0f;   // Tau threshold
float G1 = 0.0f;        // Body moment value
float G2 = 0.0f;        // Deprecated state value


// ===============================
//  NN/SVM POLICY INITIALIZATION
// ===============================
static nml_mat* X;  // STATE MATRIX TO BE INPUT INTO NN
SVM SVM_Policy_Flip;     
NN NN_Policy_Action;

float Policy_Flip = 0.0f;  
float Policy_Action = 0.0f;
float Policy_Flip_tr = 0.0f;    // Output from OC_SVM
float Policy_Action_tr = 0.0f;  // Output from NN

// ===============================
//  DEEP RL POLICY INITIALIZATION
// ===============================
static nml_mat* X;  // STATE MATRIX TO BE INPUT INTO NN
NN NN_DeepRL;
nml_mat* DeepRL_Output;

void controllerGTCInit(void)
{
    controllerGTCReset();
    controllerGTCTest();
    X = nml_mat_new(3,1);
    J = mdiag(Ixx,Iyy,Izz);

    // INIT NN/OC_SVM POLICY
    NN_init(&NN_Policy_Action,NN_Params_Flip);
    OC_SVM_init(&SVM_Policy_Flip,SVM_Params);

    // INIT DEEP RL NN POLICY
    NN_init(&NN_DeepRL,NN_Params_DeepRL);
    DeepRL_Output = nml_mat_new(2,1);

    consolePrintf("GTC Initiated\n");
}

void controllerGTCReset(void)
{
    consolePrintf("GTC Reset\n");
    consolePrintf("Policy_Type: %d\n",Policy);

    // RESET ERRORS
    e_PI = vzero();
    e_RI = vzero();

    // TURN POS/VEL CONTROLLER FLAGS ON
    kp_xf = 1.0f;
    kd_xf = 1.0f;

    // RESET SETPOINTS TO HOME POSITION
    x_d = mkvec(0.0f,0.0f,0.4f);
    v_d = mkvec(0.0f,0.0f,0.0f);
    a_d = mkvec(0.0f,0.0f,0.0f);
    b1_d = mkvec(1.0f,0.0f,0.0f);

    // RESET SYSTEM FLAGS
    tumbled = false;
    motorstop_flag = false;
    customThrust_flag = false;
    customPWM_flag = false;

    moment_flag = false;
    policy_armed_flag = false;
    flip_flag = false;
    onceFlag = false;

    // RESET TRAJECTORY VALUES
    execute_vel_traj = false;
    execute_P2P_traj = false;
    execute_GZ_vel_traj = false;
    P2P_traj_flag = vzero();
    s_0_t = vzero();
    s_f_t = vzero();
    v_t = vzero();
    a_t = vzero();
    T = vzero();
    t_traj = vzero();

    // RESET LOGGED FLIP VALUES
    statePos_tr = vzero();
    stateVel_tr = vzero();
    stateQuat_tr = mkquat(0.0f,0.0f,0.0f,1.0f);
    stateOmega_tr = vzero();

    Tau_tr = 0.0f;
    Theta_x_tr = 0.0f;
    Theta_y_tr = 0.0f;
    D_perp_tr = 0.0f;
    
    // RL - PARAMETER ESTIMATION
    Tau_thr = 0.0f;
    G1 = 0.0f;


    // SUPERVISED NN/OC_SVM COMBINATION
    Policy_Flip = 0.0f;
    Policy_Flip_tr = 0.0f;
    Policy_Action_tr = 0.0f;

}

bool controllerGTCTest(void)
{   
    return true;
}



void controllerGTC(control_t *control, setpoint_t *setpoint,
                                        sensorData_t *sensors,
                                        state_t *state,
                                        const uint32_t tick)
{
    
   

    if (RATE_DO_EXECUTE(RATE_500_HZ, tick)) {

        


        compressStates();
        compressSetpoints();
        compressFlipStates();

    }

    
    

    

}


void controlOutput(state_t *state, sensorData_t *sensors)
{

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
    
    // =========== STATE SETPOINTS =========== //
    omega_d = mkvec(0.0f,0.0f,0.0f);    // Omega-desired [rad/s]
    domega_d = mkvec(0.0f,0.0f,0.0f);   // Omega-Accl. [rad/s^2]

    eul_d = mkvec(0.0f,0.0f,0.0f);
    quat_d = rpy2quat(eul_d);           // Desired orientation from eul angles [ZYX NOTATION]

    // =========== ROTATION MATRIX =========== //
    // R changes Body axes to be in terms of Global axes
    // https://www.andre-gaschler.com/rotationconverter/
    R = quat2rotmat(stateQuat); // Quaternion to Rotation Matrix Conversion
    b3 = mvmul(R, e_3);         // Current body vertical axis in terms of global axes | [b3 = R*e_3] 
        


    // =========== TRANSLATIONAL EFFORT =========== //
    e_x = vsub(statePos, x_d); // [e_x = pos-x_d]
    e_v = vsub(stateVel, v_d); // [e_v = vel-v_d]


    // POS. INTEGRAL ERROR
    e_PI.x += (e_x.x)*dt;
    e_PI.x = clamp(e_PI.x, -i_range_xy, i_range_xy);

    e_PI.y += (e_x.y)*dt;
    e_PI.y = clamp(e_PI.y, -i_range_xy, i_range_xy);

    e_PI.z += (e_x.z)*dt;
    e_PI.z = clamp(e_PI.z, -i_range_z, i_range_z);

    /* [F_thrust_ideal = -kp_x*e_x*(kp_x_flag) + -kd_x*e_v + -kI_x*e_PI*(kp_x_flag) + m*g*e_3 + m*a_d] */
    temp1_v = veltmul(vneg(Kp_p), e_x);
    temp1_v = vscl(kp_xf,temp1_v);
    temp2_v = veltmul(vneg(Kd_p), e_v);
    temp1_v = vscl(kd_xf,temp1_v);
    temp3_v = veltmul(vneg(Ki_p), e_PI);
    P_effort = vadd3(temp1_v,temp2_v,temp3_v);
    temp1_v = vscl(m*g, e_3); // Feed-forward term
    temp2_v = vscl(m, a_d);

    F_thrust_ideal = vadd3(P_effort, temp1_v,temp2_v); 

    // =========== DESIRED BODY AXES =========== // 
    b3_d = vnormalize(F_thrust_ideal);
    b2_d = vnormalize(vcross(b3_d, b1_d));      // [b3_d x b1_d] | body-fixed horizontal axis
    temp1_v = vnormalize(vcross(b2_d, b3_d));
    R_d = mcolumns(temp1_v, b2_d, b3_d);        // Desired rotation matrix from calculations

    // =========== ROTATIONAL ERRORS =========== // 
    RdT_R = mmul(mtranspose(R_d), R);       // [R_d'*R]
    RT_Rd = mmul(mtranspose(R), R_d);       // [R'*R_d]

    temp1_v = dehat(msub(RdT_R, RT_Rd));    // [dehat(R_d'*R - R'*R)]
    e_R = vscl(0.5f, temp1_v);              // Rotation error | [eR = 0.5*dehat(R_d'*R - R'*R)]

    temp1_v = mvmul(RT_Rd, omega_d);        // [R'*R_d*omega_d]
    e_w = vsub(stateOmega, temp1_v);        // Ang. vel error | [e_w = omega - R'*R_d*omega_d] 

    // ROT. INTEGRAL ERROR
    e_RI.x += (e_R.x)*dt;
    e_RI.x = clamp(e_RI.x, -i_range_R_xy, i_range_R_xy);

    e_RI.y += (e_R.y)*dt;
    e_RI.y = clamp(e_RI.y, -i_range_R_xy, i_range_R_xy);

    e_RI.z += (e_R.z)*dt;
    e_RI.z = clamp(e_RI.z, -i_range_R_z, i_range_R_z);

    // =========== CONTROL EQUATIONS =========== // 
    /* [M = -kp_R*e_R - kd_R*e_w -ki_R*e_RI + Gyro_dyn] */

    temp1_v = veltmul(vneg(Kp_R), e_R);     // [-kp_R*e_R]
    temp2_v = veltmul(vneg(Kd_R), e_w);     // [-kd_R*e_w]
    temp3_v = veltmul(vneg(Ki_R), e_RI);    // [-ki_R*e_RI]
    R_effort = vadd3(temp1_v,temp2_v,temp3_v);

    /* Gyro_dyn = [omega x (J*omega)] - [J*( hat(omega)*R'*R_d*omega_d - R'*R_d*domega_d )] */
    temp1_v = vcross(stateOmega, mvmul(J, stateOmega)); // [omega x J*omega]


    temp1_m = mmul(hat(stateOmega), RT_Rd); //  hat(omega)*R'*R_d
    temp2_v = mvmul(temp1_m, omega_d);      // (hat(omega)*R'*R_d)*omega_d
    temp3_v = mvmul(RT_Rd, domega_d);       // (R'*R_d*domega_d)

    temp4_v = mvmul(J, vsub(temp2_v, temp3_v)); // J*(hat(omega)*R'*R_d*omega_d - R'*R_d*domega_d)
    Gyro_dyn = vsub(temp1_v,temp4_v);

    F_thrust = vdot(F_thrust_ideal, b3);    // Project ideal thrust onto b3 vector [N]
    M = vadd(R_effort,Gyro_dyn);            // Control moments [Nm]

}


void compressStates(){
    StatesZ_GTC.xy = compressXY(statePos.x,statePos.y);
    StatesZ_GTC.z = (int16_t)(statePos.z * 1000.0f);

    StatesZ_GTC.vxy = compressXY(stateVel.x, stateVel.y);
    StatesZ_GTC.vz = (int16_t)(stateVel.z * 1000.0f);

    StatesZ_GTC.wxy = compressXY(stateOmega.x/10,stateOmega.y/10);
    StatesZ_GTC.wz = (int16_t)(stateOmega.z * 1000.0f);


    float const q[4] = {
        stateQuat.x,
        stateQuat.y,
        stateQuat.z,
        stateQuat.w};
    StatesZ_GTC.quat = quatcompress(q);

    // COMPRESS SENSORY VALUES
    StatesZ_GTC.OF_xy = compressXY(Theta_x,Theta_y);
    StatesZ_GTC.Tau = (int16_t)(Tau * 1000.0f); 
    StatesZ_GTC.D_perp = (int16_t)(D_perp * 1000.0f);

    // COMPRESS THRUST/MOMENT VALUES
    StatesZ_GTC.FMz = compressXY(F_thrust,M.z*1000.0f);
    StatesZ_GTC.Mxy = compressXY(M.x*1000.0f,M.y*1000.0f);

    // COMPRESS MOTOR THRUST VALUES
    StatesZ_GTC.M_thrust12 = compressXY(M1_thrust,M2_thrust);
    StatesZ_GTC.M_thrust34 = compressXY(M3_thrust,M4_thrust);

    
    // COMPRESS PWM VALUES
    StatesZ_GTC.MS_PWM12 = compressXY(M1_pwm*0.5e-3f,M2_pwm*0.5e-3f);
    StatesZ_GTC.MS_PWM34 = compressXY(M3_pwm*0.5e-3f,M4_pwm*0.5e-3f);

    StatesZ_GTC.NN_FP = compressXY(Policy_Flip,0.0); // Flip value (OC_SVM) and Flip action (NN)

}




void compressSetpoints(){
    setpointZ_GTC.xy = compressXY(x_d.x,x_d.y);
    setpointZ_GTC.z = (int16_t)(x_d.z * 1000.0f);

    setpointZ_GTC.vxy = compressXY(v_d.x,v_d.y);
    setpointZ_GTC.vz = (int16_t)(v_d.z * 1000.0f);

    setpointZ_GTC.axy = compressXY(a_d.x,a_d.y);
    setpointZ_GTC.az = (int16_t)(a_d.z * 1000.0f);
}


void compressFlipStates(){
    FlipStatesZ_GTC.xy = compressXY(statePos_tr.x,statePos_tr.y);
    FlipStatesZ_GTC.z = (int16_t)(statePos_tr.z * 1000.0f);

    FlipStatesZ_GTC.vxy = compressXY(stateVel_tr.x, stateVel_tr.y);
    FlipStatesZ_GTC.vz = (int16_t)(stateVel_tr.z * 1000.0f);

    FlipStatesZ_GTC.wxy = compressXY(stateOmega_tr.x,stateOmega_tr.y);
    FlipStatesZ_GTC.wz = (int16_t)(stateOmega_tr.z * 1000.0f);


    float const q[4] = {
        stateQuat_tr.x,
        stateQuat_tr.y,
        stateQuat_tr.z,
        stateQuat_tr.w};
    FlipStatesZ_GTC.quat = quatcompress(q);

   FlipStatesZ_GTC.OF_xy = compressXY(Theta_x_tr,Theta_y_tr);
   FlipStatesZ_GTC.Tau = (int16_t)(Tau_tr * 1000.0f); 
   FlipStatesZ_GTC.D_perp = (int16_t)(D_perp_tr * 1000.0f);

   FlipStatesZ_GTC.NN_FP = compressXY(Policy_Flip_tr,Policy_Action_tr); // Flip value (OC_SVM) and Flip action (NN)


}