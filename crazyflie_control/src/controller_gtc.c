// CF HEADERS
#include "controller_gtc.h"
#include "NN_funcs.h"
#include "CompressedStates.h"

// =================================
//    CONTROL GAIN INITIALIZATION
// =================================
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


// SYSTEM PARAMETERS
float m = 0.0376;           // [kg]
float g = 9.81f;            // [m/s^2]
float h_ceiling = 2.10f;    // [m]
static struct mat33 J; // Rotational Inertia Matrix [kg*m^2]


static float d = 0.040f;    // COM to Prop [m]
static float dp = 0.028284; // COM to Prop along x-axis [m]
                            // [dp = d*sin(45 deg)]

static float const kf = 2.2e-8f;    // Thrust Coeff [N/(rad/s)^2]
static float const c_tf = 0.00618f; // Moment Coeff [Nm/N]
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
struct vec stateEul = {0.0f,0.0f,0.0f}; // Pose in Euler Angles [YZX Notation]


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
bool safeModeFlag = false;

bool execute_traj = false;
bool policy_armed_flag = false;

bool flip_flag = false;
bool onceFlag = false;

bool moment_flag = false;
bool attCtrlEnable = false;
bool safeModeEnable = true;


// DEFINE POLICY TYPE ACTIVATED

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
float Tau_thr = 0.0f;   // Tau threshold
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
    controllerGTCReset();
    controllerGTCTest();
    X = nml_mat_new(3,1);
    J = mdiag(1.65717e-5f,1.65717e-5f,2.92617e-5f);

    initNN_Layers(&Scaler_Flip,W_flip,b_flip,NN_Params_Flip,3);
    initNN_Layers(&Scaler_Policy,W_policy,b_policy,NN_Params_Policy,3);
    consolePrintf("GTC Initiated\n");
}

void controllerGTCReset(void)
{
    consolePrintf("GTC Reset\n");

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

    // RESET SYSTEM FLAGS
    tumbled = false;
    motorstop_flag = false;

    moment_flag = false;
    policy_armed_flag = false;
    flip_flag = false;
    onceFlag = false;

    // RESET TRAJECTORY VALUES
    execute_traj = false;
    s_0_t = vzero();
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
    OFx_tr = 0.0f;
    OFy_tr = 0.0f;
    RREV_tr = 0.0f;
    d_ceil_tr = 0.0f;
    
    NN_tr_flip = 0.0f;
    NN_tr_policy = 0.0f;

    Tau_thr = 0.0f;
    G1 = 0.0f;

}

bool controllerGTCTest(void)
{   
    return true;
}

void GTC_Command(setpoint_t *setpoint)
{   
    switch(setpoint->cmd_type){
        case 0: // Reset
            controllerGTCReset();
            break;


        case 1: // Position
            x_d.x = setpoint->cmd_val1;
            x_d.y = setpoint->cmd_val2;
            x_d.z = setpoint->cmd_val3;
            kp_xf = setpoint->cmd_flag;
            break;


        case 2: // Velocity
            v_d.x = setpoint->cmd_val1;
            v_d.y = setpoint->cmd_val2;
            v_d.z = setpoint->cmd_val3;
            kd_xf = setpoint->cmd_flag;
            break;


        case 3: // Acceleration
            a_d.x = setpoint->cmd_val1;
            a_d.y = setpoint->cmd_val2;
            a_d.z = setpoint->cmd_val3;
            break;

        case 4: // Tumble-Detection
            tumble_detection = setpoint->cmd_flag;
            break;

        case 5: // Hard Set All Motorspeeds to Zero
            motorstop_flag = true;
            break;
        
        case 6: // Reset ROS Parameters
            
            break;

        case 7: // Execute Moment-Based Flip

            M_d.x = setpoint->cmd_val1*1e-3;
            M_d.y = setpoint->cmd_val2*1e-3;
            M_d.z = setpoint->cmd_val3*1e-3;

            moment_flag = (bool)setpoint->cmd_flag;
            break;

        case 8: // Arm Policy Maneuver
            Tau_thr = setpoint->cmd_val1;
            G1 = setpoint->cmd_val2;
            G2 = setpoint->cmd_val3;

            policy_armed_flag = setpoint->cmd_flag;

            break;
            
        case 9: // Trajectory Values
            traj_type = (axis_direction)setpoint->cmd_flag;

            switch(traj_type){

                case x:

                    s_0_t.x = setpoint->cmd_val1;               // Starting position [m]
                    v_t.x = setpoint->cmd_val2;                 // Desired velocity [m/s]
                    a_t.x = setpoint->cmd_val3;                 // Max acceleration [m/s^2]
                    T.x = (a_t.x+fsqr(v_t.x))/(a_t.x*v_t.x);    // Find trajectory manuever time [s]

                    t_traj.x = 0.0f; // Reset timer
                    execute_traj = true;
                    break;

                case y:

                    s_0_t.y = setpoint->cmd_val1;
                    v_t.y = setpoint->cmd_val2;
                    a_t.y = setpoint->cmd_val3;
                    T.y = (a_t.y+fsqr(v_t.y))/(a_t.y*v_t.y); 

                    t_traj.y = 0.0f;
                    execute_traj = true;
                    break;

                case z:

                    s_0_t.z = setpoint->cmd_val1;
                    v_t.z = setpoint->cmd_val2;
                    a_t.z = setpoint->cmd_val3;
                    T.z = (a_t.z+fsqr(v_t.z))/(a_t.z*v_t.z); 

                    t_traj.z = 0.0f;
                    execute_traj = true;
                    break;
                    
            }

            break;
    }
    
}

void controllerGTCTraj()
{
   for (int i = 0; i < 3; i++)
    {
    
        if(t_traj.idx[i]<=v_t.idx[i]/a_t.idx[i]) // t <= v/a
        {
            x_d.idx[i] = 0.5f*a_t.idx[i]*t_traj.idx[i]*t_traj.idx[i] + s_0_t.idx[i]; // 0.5*a*t^2 + s_0
            v_d.idx[i] = a_t.idx[i]*t_traj.idx[i]; // a*t
            a_d.idx[i] = a_t.idx[i]; // a

        }

        // CONSTANT VELOCITY TRAJECTORY
        if(v_t.idx[i]/a_t.idx[i] < t_traj.idx[i]) // v/a < t
        {
            x_d.idx[i] = v_t.idx[i]*t_traj.idx[i] - fsqr(v_t.idx[i])/(2.0f*a_t.idx[i]) + s_0_t.idx[i]; // v*t - (v/(2*a))^2 + s_0
            v_d.idx[i] = v_t.idx[i]; // v
            a_d.idx[i] = 0.0f;
        }

        // // POINT-TO-POINT (1m) POSITION TRAJECTORY
        // if(v_t.idx[i]/a_t.idx[i] <= t_traj.idx[i] && t_traj.idx[i] < (T.idx[i]-v_t.idx[i]/a_t.idx[i])) // v/a < t < (T-v/a)
        // {
        //     x_d.idx[i] = v_t.idx[i]*t_traj.idx[i] - fsqr(v_t.idx[i])/(2.0f*a_t.idx[i]) + s_0_t.idx[i]; // v*t - (v/2*a)^2 = s_0
        //     v_d.idx[i] = v_t.idx[i]; // v
        //     a_d.idx[i] = 0.0f; 
        // }

        // if((T.idx[i]-v_t.idx[i]/a_t.idx[i]) < t_traj.idx[i] && t_traj.idx[i] <= T.idx[i]) // (T-v/a) < t < T
        // {
        //     x_d.idx[i] = (2.0f*a_t.idx[i]*v_t.idx[i]*T.idx[i]-2.0f*fsqr(v_t.idx[i])-fsqr(a_t.idx[i])*fsqr(t_traj.idx[i]-T.idx[i]))/(2.0f*a_t.idx[i]) + s_0_t.idx[i];
        //     v_d.idx[i] = a_t.idx[i]*(T.idx[i]-t_traj.idx[i]);
        //     a_d.idx[i] = -a_t.idx[i];
        // }


        t_traj.idx[i] += dt;
    }

    
}


void controllerGTC(control_t *control, setpoint_t *setpoint,
                                        sensorData_t *sensors,
                                        state_t *state,
                                        const uint32_t tick)
{
    if (RATE_DO_EXECUTE(RATE_500_HZ, tick)) {

        if (setpoint->GTC_cmd_rec == true)
            {
                GTC_Command(setpoint);
                setpoint->GTC_cmd_rec = false;
            }

        if (errorReset){
            controllerGTCReset();
            errorReset = false;
            }

        if(execute_traj){
            controllerGTCTraj();
        }

        d_ceil = 2.10f-state->position.z;
        Tau = d_ceil/state->velocity.z;
        OFx = -state->velocity.y/d_ceil;
        OFy = -state->velocity.x/d_ceil;
        RREV = state->velocity.z/d_ceil;

        X->data[0][0] = RREV;
        X->data[1][0] = OFy;
        X->data[2][0] = d_ceil; // d_ceiling [m]

        
        controlOutput(state,sensors);

        if(policy_armed_flag == true){ 
                
            switch(POLICY_TYPE)
            {
                case RL:
                {
                    if(Tau <= Tau_thr && onceFlag == false){
                        onceFlag = true;
                        flip_flag = true;  

                        // UPDATE AND RECORD FLIP VALUES
                        statePos_tr = statePos;
                        stateVel_tr = stateVel;
                        stateQuat_tr = stateQuat;
                        stateOmega_tr = stateOmega;

                        Tau_tr = Tau;
                        OFx_tr = OFx;
                        OFy_tr = OFy;
                        RREV_tr = RREV;
                        d_ceil_tr = d_ceil;

                    
                        M_d.x = 0.0f;
                        M_d.y = -G1*1e-3f;
                        M_d.z = 0.0f;

                        F_thrust_flip = 0.0;
                        M_x_flip = M_d.x*1e3f;
                        M_y_flip = M_d.y*1e3f;
                        M_z_flip = M_d.z*1e3f;
                    }
                    break;
                }

                case NN:
                {   
                    NN_flip = NN_Forward_Flip(X,&Scaler_Flip,W_flip,b_flip);
                    NN_policy = -NN_Forward_Policy(X,&Scaler_Policy,W_policy,b_policy);

                    if(NN_flip >= 0.9 && onceFlag == false)
                    {   
                        onceFlag = true;
                        flip_flag = true;

                        // UPDATE AND RECORD FLIP VALUES
                        statePos_tr = statePos;
                        stateVel_tr = stateVel;
                        stateQuat_tr = stateQuat;
                        stateOmega_tr = stateOmega;

                        Tau_tr = Tau;
                        OFx_tr = OFx;
                        OFy_tr = OFy;
                        RREV_tr = RREV;
                        d_ceil_tr = d_ceil;

                        NN_tr_flip = NN_Forward_Flip(X,&Scaler_Flip,W_flip,b_flip);
                        NN_tr_policy = NN_Forward_Policy(X,&Scaler_Policy,W_policy,b_policy);


                        M_d.x = 0.0f;
                        M_d.y = -NN_tr_policy*1e-3f;
                        M_d.z = 0.0f;

                        F_thrust_flip = 0.0;
                        M_x_flip = M_d.x*1e3f;
                        M_y_flip = M_d.y*1e3f;
                        M_z_flip = M_d.z*1e3f;
                    }

                    break;
                }

            }

            
            if(flip_flag == true)
            {
                // Doubling front motor values and zeroing back motors is
                // equal to increasing front motors and decreasing back motors.
                // This gives us the highest possible moment and avoids PWM cutoff issue
                M = vscl(2.0f,M_d); 
                F_thrust = 0.0f;
            }
        }
        if(moment_flag == true)
        {
            F_thrust = 0.0f;
            M = M_d;
        }

        // =========== CONVERT THRUSTS AND MOMENTS TO PWM =========== // 
        f_thrust_g = F_thrust/4.0f*Newton2g;
        f_roll_g = M.x/(4.0f*dp)*Newton2g;
        f_pitch_g = M.y/(4.0f*dp)*Newton2g;
        f_yaw_g = M.z/(4.0*c_tf)*Newton2g;

        f_thrust_g = clamp(f_thrust_g,0.0,f_MAX*0.8);    // Clamp thrust to prevent control saturation

        // Add respective thrust components and limit to (0 <= PWM <= 60,000)
        M1_pwm = limitPWM(thrust2PWM(f_thrust_g + f_roll_g - f_pitch_g + f_yaw_g)); 
        M2_pwm = limitPWM(thrust2PWM(f_thrust_g + f_roll_g + f_pitch_g - f_yaw_g));
        M3_pwm = limitPWM(thrust2PWM(f_thrust_g - f_roll_g + f_pitch_g + f_yaw_g));
        M4_pwm = limitPWM(thrust2PWM(f_thrust_g - f_roll_g - f_pitch_g - f_yaw_g));


        // TUMBLE DETECTION
        if (b3.z <= 0 && tumble_detection == true){
            tumbled = true;
        }
        
        if(motorstop_flag || tumbled){ // Cutoff all motor values
            f_thrust_g = 0.0f;
            M1_pwm = 0;
            M2_pwm = 0;
            M3_pwm = 0;
            M4_pwm = 0;
        }



        // THESE CONNECT TO POWER_DISTRIBUTION_STOCK.C TO 
        control->thrust = f_thrust_g;
        control->roll = (int16_t)(f_roll_g*1e3f);
        control->pitch = (int16_t)(f_pitch_g*1e3f);
        control->yaw = (int16_t)(f_yaw_g*1e3f);
       


        

        compressStates();
        compressSetpoints();
        compressFlipStates();
    }

        
    if(safeModeEnable) // If safeMode is enabled then override all PWM commands
    {
        M1_pwm = 0;
        M2_pwm = 0;
        M3_pwm = 0;
        M4_pwm = 0;
    }
    else
    {
        motorsSetRatio(MOTOR_M1, M4_pwm);
        motorsSetRatio(MOTOR_M2, M3_pwm);
        motorsSetRatio(MOTOR_M3, M2_pwm);
        motorsSetRatio(MOTOR_M4, M1_pwm);
    }

    

}






void compressStates(){
    StatesZ_GTC.xy = compressXY(statePos.x,statePos.y);
    StatesZ_GTC.z = (int16_t)(statePos.z * 1000.0f);

    StatesZ_GTC.vxy = compressXY(stateVel.x, stateVel.y);
    StatesZ_GTC.vz = (int16_t)(stateVel.z * 1000.0f);

    StatesZ_GTC.wxy = compressXY(stateOmega.x,stateOmega.y);
    StatesZ_GTC.wz = (int16_t)(stateOmega.z * 1000.0f);


    float const q[4] = {
        stateQuat.x,
        stateQuat.y,
        stateQuat.z,
        stateQuat.w};
    StatesZ_GTC.quat = quatcompress(q);

    // COMPRESS SENSORY VALUES
    StatesZ_GTC.OF_xy = compressXY(OFx,OFy);
    StatesZ_GTC.Tau = (int16_t)(Tau * 1000.0f); 
    StatesZ_GTC.d_ceil = (int16_t)(d_ceil * 1000.0f);

    // COMPRESS THRUST/MOMENT VALUES
    StatesZ_GTC.FMz = compressXY(F_thrust,M.z*1000.0f);
    StatesZ_GTC.Mxy = compressXY(M.x*1000.0f,M.y*1000.0f);

    // COMPRESS PWM VALUES
    StatesZ_GTC.MS_PWM12 = compressXY(M1_pwm*0.5e-3f,M2_pwm*0.5e-3f);
    StatesZ_GTC.MS_PWM34 = compressXY(M3_pwm*0.5e-3f,M4_pwm*0.5e-3f);

    StatesZ_GTC.NN_FP = compressXY(NN_flip,NN_policy);

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

   FlipStatesZ_GTC.OF_xy = compressXY(OFx_tr,OFy_tr);
   FlipStatesZ_GTC.Tau = (int16_t)(Tau_tr * 1000.0f); 
   FlipStatesZ_GTC.d_ceil = (int16_t)(d_ceil_tr * 1000.0f);

   FlipStatesZ_GTC.NN_FP = compressXY(NN_tr_flip,NN_tr_policy);

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


// // PARAMETER GROUPS
// PARAM_GROUP_START(GTC_Params)
// PARAM_ADD(PARAM_FLOAT, P_kp_xy, &P_kp_xy)
// PARAM_ADD(PARAM_FLOAT, P_kp_z,  &P_kp_z)
// PARAM_ADD(PARAM_FLOAT, P_kd_xy, &P_kd_xy) 
// PARAM_ADD(PARAM_FLOAT, i_range_xy, &i_range_xy)
// PARAM_ADD(PARAM_FLOAT, P_kd_z,  &P_kd_z)
// PARAM_ADD(PARAM_FLOAT, P_ki_xy, &P_ki_xy)
// PARAM_ADD(PARAM_FLOAT, P_ki_z,  &P_ki_z)
// PARAM_ADD(PARAM_FLOAT, i_range_z, &i_range_z)

// PARAM_ADD(PARAM_FLOAT, R_kp_xy, &R_kp_xy)
// PARAM_ADD(PARAM_FLOAT, R_kd_xy, &R_kd_xy) 
// PARAM_ADD(PARAM_FLOAT, R_ki_xy, &R_ki_xy)
// PARAM_ADD(PARAM_FLOAT, i_range_R_xy, &i_range_R_xy)

// PARAM_ADD(PARAM_FLOAT, R_kp_z,  &R_kp_z)
// PARAM_ADD(PARAM_FLOAT, R_kd_z,  &R_kd_z)
// PARAM_ADD(PARAM_FLOAT, R_ki_z,  &R_ki_z)
// PARAM_ADD(PARAM_FLOAT, i_range_R_z, &i_range_R_z)

// PARAM_ADD(PARAM_FLOAT, b1_d_x, &b1_d.x)
// PARAM_ADD(PARAM_FLOAT, b1_d_y, &b1_d.y)
// PARAM_ADD(PARAM_FLOAT, b1_d_z, &b1_d.z)

// PARAM_ADD(PARAM_FLOAT, CF_mass, &m)

// // PARAM_ADD(PARAM_UINT8, AttCtrl, &attCtrlEnable)
// // PARAM_ADD(PARAM_UINT8, Tumbled, &tumbled)
// // PARAM_ADD(PARAM_UINT8, Error_Reset, &errorReset)
// PARAM_ADD(PARAM_UINT8, SafeModeFlag, &safeModeEnable)

// PARAM_GROUP_STOP(GTC_Params)


// // LOGGING GROUPS

// LOG_GROUP_START(LogStateData_GTC)
// LOG_ADD(LOG_UINT32, Z_xy,   &StatesZ_GTC.xy)
// LOG_ADD(LOG_INT16,  Z_z,    &StatesZ_GTC.z)

// LOG_ADD(LOG_UINT32, Z_vxy,  &StatesZ_GTC.vxy)
// LOG_ADD(LOG_INT16,  Z_vz,   &StatesZ_GTC.vz)

// LOG_ADD(LOG_UINT32, Z_quat, &StatesZ_GTC.quat)

// LOG_ADD(LOG_UINT32, Z_wxy,  &StatesZ_GTC.wxy)
// LOG_ADD(LOG_INT16,  Z_wz,   &StatesZ_GTC.wz)

// LOG_ADD(LOG_UINT32, Z_OFxy, &StatesZ_GTC.OF_xy)
// LOG_ADD(LOG_INT16,  Z_Tau,  &StatesZ_GTC.Tau)
// LOG_ADD(LOG_INT16,  Z_d_ceil, &StatesZ_GTC.d_ceil)

// LOG_ADD(LOG_UINT32, Z_FMz, &StatesZ_GTC.FMz)
// LOG_ADD(LOG_UINT32, Z_Mxy, &StatesZ_GTC.Mxy)

// LOG_ADD(LOG_UINT32, Z_PWM12, &StatesZ_GTC.MS_PWM12)
// LOG_ADD(LOG_UINT32, Z_PWM34, &StatesZ_GTC.MS_PWM34)

// LOG_ADD(LOG_UINT32, Z_NN_FP, &StatesZ_GTC.NN_FP)
// LOG_GROUP_STOP(LogStateData_GTC)


// LOG_GROUP_START(LogSetPoints_GTC)
// LOG_ADD(LOG_UINT32, Z_xy,   &setpointZ_GTC.xy)
// LOG_ADD(LOG_INT16,  Z_z,    &setpointZ_GTC.z)

// LOG_ADD(LOG_UINT32, Z_vxy,  &setpointZ_GTC.vxy)
// LOG_ADD(LOG_INT16,  Z_vz,   &setpointZ_GTC.vz)

// LOG_ADD(LOG_UINT32, Z_axy,  &setpointZ_GTC.axy)
// LOG_ADD(LOG_INT16,  Z_az,   &setpointZ_GTC.az)
// LOG_GROUP_STOP(LogSetPoints_GTC)


// LOG_GROUP_START(LogFlipData_GTC)
// LOG_ADD(LOG_UINT32, Z_xy,   &FlipStatesZ_GTC.xy)
// LOG_ADD(LOG_INT16,  Z_z,    &FlipStatesZ_GTC.z)

// LOG_ADD(LOG_UINT32, Z_vxy,  &FlipStatesZ_GTC.vxy)
// LOG_ADD(LOG_INT16,  Z_vz,   &FlipStatesZ_GTC.vz)

// LOG_ADD(LOG_UINT32, Z_quat, &FlipStatesZ_GTC.quat)

// LOG_ADD(LOG_UINT32, Z_wxy,  &FlipStatesZ_GTC.wxy)
// LOG_ADD(LOG_INT16,  Z_wz,   &FlipStatesZ_GTC.wz)

// LOG_ADD(LOG_UINT32, Z_OFxy, &FlipStatesZ_GTC.OF_xy)
// LOG_ADD(LOG_INT16,  Z_Tau,  &FlipStatesZ_GTC.Tau)
// LOG_ADD(LOG_INT16,  Z_d_ceil, &FlipStatesZ_GTC.d_ceil)

// LOG_ADD(LOG_UINT8, Flip_Flag, &flip_flag)
// LOG_GROUP_STOP(LogFlipData_GTC)


// LOG_GROUP_START(valsLog)
// LOG_ADD(LOG_UINT8, Motorstop_Flag, &motorstop_flag)
// LOG_ADD(LOG_FLOAT, Pos_Ctrl_Flag, &kp_xf)
// LOG_ADD(LOG_FLOAT, Vel_Ctrl_Flag, &kd_xf)
// LOG_ADD(LOG_UINT8, Execute_Traj_Flag, &execute_traj)
// LOG_ADD(LOG_UINT8, Tumbled_Flag, &tumbled)
// LOG_ADD(LOG_UINT8, Moment_Flag, &moment_flag)
// LOG_ADD(LOG_UINT8, Policy_Armed_Flag, &policy_armed_flag)
// LOG_GROUP_STOP(valsLog)
