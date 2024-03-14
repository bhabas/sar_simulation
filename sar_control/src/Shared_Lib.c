#include "Shared_Lib.h"



// =================================
//    INITIAL SYSTEM PARAMETERS
// =================================
float m = 0.00f;            // [kg]
float Ixx = 0.00f;          // [kg*m^2]
float Iyy = 0.00f;          // [kg*m^2]
float Izz = 0.00f;          // [kg*m^2]
struct mat33 J;             // Rotational Inertia Matrix [kg*m^2]

float C_tf = 0.0f;          // Moment Coeff [Nm/N]
float Thrust_max = 0.0f;         // Max thrust per motor [g]

const float g = 9.81f;                        // Gravity [m/s^2]
const struct vec e_3 = {0.0f, 0.0f, 1.0f};    // Global z-axis

float dt = (float)(1.0f/RATE_100_HZ);
uint32_t PrevCrazyswarmTick = 0;
uint32_t prev_tick = 0;
struct CTRL_CmdPacket CTRL_Cmd;
SAR_Types SAR_Type = SAR_NONE;


// =================================
//       GEOMETRIC PARAMETERS
// =================================

float Prop_14_x = 0.0f;         // Front Prop Distance - x-axis [m]
float Prop_14_y = 0.0f;         // Front Prop Distance - y-axis [m]
float Prop_23_x = 0.0f;         // Rear  Prop Distance - x-axis [m]
float Prop_23_y = 0.0f;         // Rear  Prop Distance - y-axis [m]

float L_eff = 0.0f;             // Effective Leg Length [m]
float Forward_Reach = 0.0f;     // Forward Reach [m]
float Collision_Radius = 0.0f;  // Collision Radius [m]


// =================================
//    CONTROL GAIN INITIALIZATION
// =================================
// XY POSITION PID
float P_kp_xy = 0.0f;
float P_kd_xy = 0.0f;
float P_ki_xy = 0.0f;
float i_range_xy = 0.0f;

// Z POSITION PID
float P_kp_z = 0.0f;
float P_kd_z = 0.0f;
float P_ki_z = 0.0f;
float i_range_z = 0.0f;

// XY ATTITUDE PID
float R_kp_xy = 0.0f;
float R_kd_xy = 0.0f;
float R_ki_xy = 0.0f;
float i_range_R_xy = 0.0f;

// Z ATTITUDE PID
float R_kp_z = 0.0f;
float R_kd_z = 0.0f;
float R_ki_z = 0.0f;
float i_range_R_z = 0.0f;


// INIT CTRL GAIN VECTORS 
struct vec Kp_p;    // Pos. Proportional Gains 
struct vec Kd_p;    // Pos. Derivative Gains
struct vec Ki_p;    // Pos. Integral Gains  

struct vec Kp_R;    // Rot. Proportional Gains
struct vec Kd_R;    // Rot. Derivative Gains
struct vec Ki_R;    // Rot. Integral Gains


// INIT GAIN FLAGS
float kp_xf = 1;    // Pos. Gain Flag
float kd_xf = 1;    // Pos. Derivative Gain Flag


// =================================
//     BODY WRT ORIGIN STATES
// =================================
struct vec Pos_B_O = {0.0f,0.0f,0.0f};          // Pos [m]
struct vec Vel_B_O = {0.0f,0.0f,0.0f};          // Vel [m/s]
struct vec Accel_B_O = {0.0f,0.0f,0.0f};        // Linear Accel. [m/s^2]
float Accel_B_O_Mag = 0.0f;                     // Linear Acceleration Magnitude [m/s^2]

struct quat Quat_B_O = {0.0f,0.0f,0.0f,1.0f};   // Orientation
struct vec Omega_B_O = {0.0f,0.0f,0.0f};        // Angular Rate [rad/s]
struct vec Omega_B_O_prev  = {0.0f,0.0f,0.0f};  // Prev Angular Rate [rad/s^2]
struct vec dOmega_B_O = {0.0f,0.0f,0.0f};       // Angular Accel [rad/s^2]

struct mat33 R;                                 // Orientation as rotation matrix
struct vec b3 = {0.0f,0.0f,1.0f};               // Current body z-axis in global coord.

// =================================
//     BODY WRT PLANE STATES
// =================================
struct vec Pos_P_B = {0.0f,0.0f,0.0f};          // Pos [m]
struct vec Vel_B_P = {0.0f,0.0f,0.0f};          // Vel [m/s]
struct quat Quat_P_B = {0.0f,0.0f,0.0f,1.0f};   // Orientation
struct vec Omega_B_P = {0.0f,0.0f,0.0f};        // Angular Rate [rad/s]

// RELATIVE STATES
float D_perp = 0.0f;                            // Distance perp to plane [m]
float D_perp_CR = 0.0f;                         // Distance from CR to plane [m]
float Vel_mag_B_P = 0.0f;                       // Velocity magnitude relative [m/s]
float Vel_angle_B_P = 0.0f;                     // Velocity angle relative [deg]
                            

// =================================
//         DESIRED STATES
// =================================
struct vec x_d = {0.0f,0.0f,0.0f};              // Pos-desired [m]
struct vec v_d = {0.0f,0.0f,0.0f};              // Vel-desired [m/s]
struct vec a_d = {0.0f,0.0f,0.0f};              // Acc-desired [m/s^2]

struct vec b1_d = {1.0f,0.0f,0.0f};             // Desired body x-axis in global coord. 
struct vec b2_d = {0.0f,1.0f,0.0f};             // Desired body y-axis in global coord.
struct vec b3_d = {0.0f,0.0f,1.0f};             // Desired body z-axis in global coord.
struct mat33 R_d;                               // Desired rotational matrix from b_d vectors

struct quat quat_d = {0.0f,0.0f,0.0f,1.0f};     // Orientation-desired [qx,qy,qz,qw]
struct vec omega_d = {0.0f,0.0f,0.0f};          // Omega-desired [rad/s]
struct vec domega_d = {0.0f,0.0f,0.0f};         // Ang. Acc-desired [rad/s^2]

// =================================
//         STATE ERRORS
// =================================
struct vec e_x;  // Pos-error [m]
struct vec e_v;  // Vel-error [m/s]
struct vec e_PI; // Pos. Integral-error [m*s]

struct vec e_R;  // Rotation-error [rad]
struct vec e_w;  // Omega-error [rad/s]
struct vec e_RI; // Rot. Integral-error [rad*s]


// =================================
//       CONTROLLER ACTUATIONS
// =================================
struct vec F_thrust_ideal = {0.0f,0.0f,1.0f};   // Ideal thrust vector [N]
float F_thrust = 0.0f;                          // Implemented body thrust [N]
struct vec M = {0.0f,0.0f,0.0f};                // Implemented body moments [N*m]
struct vec M_d = {0.0f,0.0f,0.0f};              // Desired moment [N*m]

// MOTOR THRUST ACTIONS
float f_thrust_g = 0.0f;    // Motor thrust - Thrust [g]
float f_roll_g = 0.0f;      // Motor thrust - Roll   [g]
float f_pitch_g = 0.0f;     // Motor thrust - Pitch  [g]
float f_yaw_g = 0.0f;       // Motor thrust - Yaw    [g]

// INDIVIDUAL MOTOR THRUSTS
float M1_thrust = 0.0f;     // Motor 1 [g]
float M2_thrust = 0.0f;     // Motor 2 [g]
float M3_thrust = 0.0f;     // Motor 3 [g]
float M4_thrust = 0.0f;     // Motor 4 [g]

// MOTOR M_CMD VALUES
uint16_t M1_CMD = 0;        // [0 - 65,535]
uint16_t M2_CMD = 0;        // [0 - 65,535]
uint16_t M3_CMD = 0;        // [0 - 65,535]
uint16_t M4_CMD = 0;        // [0 - 65,535]

// CONTROL OVERRIDE VALUES
uint16_t M_CMD_override[4] = {0,0,0,0};               // Motor M_CMD values
float thrust_override[4] = {0.0f,0.0f,0.0f,0.0f};   // Motor thrusts [g] 


// =================================
//   TEMPORARY CALC VECS/MATRICES
// =================================
static struct vec temp1_v; 
static struct vec temp2_v;
static struct vec temp3_v;
static struct vec temp4_v;
static struct mat33 temp1_m;  

static struct vec P_effort; // Effort by positional PID
static struct vec R_effort; // Effort by rotational PID

static struct mat33 RdT_R;  // Rd' * R
static struct mat33 RT_Rd;  // R' * Rd
static struct vec Gyro_dyn;



// =================================
//        OPTICAL FLOW STATES
// =================================

// OPTICAL FLOW STATES (GROUND TRUTH)
float Tau = 0.0f;       // [s]
float Tau_CR = 0.0f;    // [s]
float Theta_x = 0.0f;   // [rad/s] 
float Theta_y = 0.0f;   // [rad/s]

// OPTICAL FLOW STATES (CAMERA ESTIMATE)
float Tau_Cam = 0.0f;       // [s]
float Theta_x_Cam = 0.0f;   // [rad/s]
float Theta_y_Cam = 0.0f;   // [rad/s]

// CAMERA PARAMETERS
float IW = 1.152e-3f;       // Image Width [m]
float IH = 1.152e-3f;       // Image Height [m]
float focal_len = 0.66e-3f; // Focal Length [m]
int32_t N_up = 160;         // Pixel Count Horizontal [m]
int32_t N_vp = 160;         // Pixel Count Vertical [m]
int32_t Cam_dt = 100;       // Time Between Images [ms]


int32_t UART_arr[UART_ARR_SIZE] = {0};
bool isOFUpdated = false;

// =================================
//  FLAGS AND SYSTEM INITIALIZATION
// =================================

// CONTROLLER FLAGS
bool Tumbled_Flag = false;
bool TumbleDetect_Flag = true;
bool MotorStop_Flag = false;
bool AngAccel_Flag = false;
bool Armed_Flag = false;
bool CustomThrust_Flag = false;
bool CustomMotorCMD_Flag = false;
uint16_t CMD_ID = 0;


// SENSOR FLAGS
bool CamActive_Flag = false;


// =================================
//       POLICY INITIALIZATION
// =================================

// DEFINE POLICY TYPE ACTIVATED
PolicyType Policy = PARAM_OPTIM;
nml_mat* X_input;       // STATE MATRIX TO BE INPUT INTO POLICY
nml_mat* Y_output;      // POLICY OUTPUT MATRIX
float Y_output_trg[4];  // POLICY OUTPUT MATRIX

// POLICY FLAGS
bool Policy_Armed_Flag = false;
bool Trg_Flag = false;
bool onceFlag = false;

// POLICY TRIGGER/ACTION VALUES
float a_Trg = 0.0f;  
float a_Rot = 0.0f;
float a_Rot_bounds[2] = {-1.0f,1.0f};

// ===============================
//  DEEP RL POLICY INITIALIZATION
// ===============================

NN NN_DeepRL;


// ==========================================
//  RECORD SYSTEM STATES AT POLICY TRIGGER
// ==========================================

// BODY WRT ORIGIN STATES
struct vec Pos_B_O_trg = {0.0f,0.0f,0.0f};         // Pos [m]
struct vec Vel_B_O_trg = {0.0f,0.0f,0.0f};         // Vel [m/s]
struct quat Quat_B_O_trg = {0.0f,0.0f,0.0f,1.0f};  // Orientation
struct vec Omega_B_O_trg = {0.0f,0.0f,0.0f};       // Angular Rate [rad/s]


// BODY WRT PLANE STATES
struct vec Pos_P_B_trg = {0.0f,0.0f,0.0f};         // Pos [m]
struct vec Vel_B_P_trg = {0.0f,0.0f,0.0f};         // Vel [m/s]
struct quat Quat_P_B_trg = {0.0f,0.0f,0.0f,1.0f};  // Orientation
struct vec Omega_B_P_trg = {0.0f,0.0f,0.0f};       // Angular Rate [rad/s]
float Vel_mag_B_P_trg = 0.0f;                      // Velocity magnitude relative [m/s]
float Vel_angle_B_P_trg = 0.0f;                    // Velocity angle relative [deg]
float D_perp_trg = 0.0f;                           // Distance perp to plane [m]
float D_perp_CR_trg = 0.0f;                        // Distance from CR to plane [m]



// OPTICAL FLOW STATES
float Tau_trg = 0.0f;               // [s]
float Tau_CR_trg = 0.0f;            // [s]
float Theta_x_trg = 0.0f;           // [rad/s]
float Theta_y_trg = 0.0f;           // [rad/s]

// OPTICAL FLOW CAMERA ESTIMATES
float Tau_Cam_trg = 0.0f;           // [rad/s]
float Theta_x_Cam_trg = 0.0f;       // [rad/s]
float Theta_y_Cam_trg = 0.0f;       // [rad/s]

// POLICY TRIGGER/ACTION VALUES
float a_Trg_trg = 0.0f;    
float a_Rot_trg = 0.0f;

// =================================
//  RECORD SYSTEM STATES AT IMPACT
// =================================
bool Impact_Flag_OB = false;
float Accel_B_O_Mag_impact_OB = 0.0f;                    // Linear Acceleration Magnitude [m/s^2]
struct vec Pos_B_O_impact_OB = {0.0f,0.0f,0.0f};         // Pos [m]
struct quat Quat_B_O_impact_OB = {0.0f,0.0f,0.0f,1.0f};  // Orientation

struct vec Vel_B_P_impact_OB = {0.0f,0.0f,0.0f};         // Vel [m/s]
struct vec Omega_B_P_impact_OB = {0.0f,0.0f,0.0f};       // Angular Rate [rad/s]

// =================================
//    LANDING SURFACE PARAMETERS
// =================================
float Plane_Angle_deg = 0.0f;           // Plane Angle [deg]
struct vec r_P_O = {0.0f,0.0f,0.0f};    // Plane Position Vector        [m]



// =================================
//         ROTATION MATRICES
// =================================
struct mat33 R_WP;                      // Rotation matrix from world to plane
struct mat33 R_PW;                      // Rotation matrix from plane to world



void CTRL_Command(struct CTRL_CmdPacket *CTRL_Cmd)
{
    // consolePrintf("Cmd ID: %d | Cmd Type: %d | Cmd Val1: %.3f | Cmd Val2: %.3f | Cmd Val3: %.3f | Cmd Flag: %.3f\n",CTRL_Cmd->cmd_ID,CTRL_Cmd->cmd_type,CTRL_Cmd->cmd_val1,CTRL_Cmd->cmd_val2,CTRL_Cmd->cmd_val3,CTRL_Cmd->cmd_flag);
    CMD_ID = CTRL_Cmd->cmd_ID;
    switch(CTRL_Cmd->cmd_type){
        case 0: // Reset
            controllerOutOfTreeReset();
            break;


        case 1: // Position
            x_d.x = CTRL_Cmd->cmd_val1;
            x_d.y = CTRL_Cmd->cmd_val2;
            x_d.z = CTRL_Cmd->cmd_val3;
            kp_xf = CTRL_Cmd->cmd_flag;
            break;
        
        case 2: // Velocity
            v_d.x = CTRL_Cmd->cmd_val1;
            v_d.y = CTRL_Cmd->cmd_val2;
            v_d.z = CTRL_Cmd->cmd_val3;
            kd_xf = CTRL_Cmd->cmd_flag;
            break;

        case 3: 

            /* Do Nothing */
            break;

        case 4: // Euler Angle

            // TODO: ADD ANGLE SETPOINT OPTION INTO CONTROLLER FOR ANGLE BASED POLICY

            break;        

        case 5: // Hard Set All Motorspeeds to Zero
            MotorStop_Flag = !MotorStop_Flag;
            break;

        case 7: // Execute Moment Corresponding to Angular Acceleration

            M_d.x = CTRL_Cmd->cmd_val1*Ixx;
            M_d.y = CTRL_Cmd->cmd_val2*Iyy;
            M_d.z = CTRL_Cmd->cmd_val3*Izz;

            AngAccel_Flag = (bool)CTRL_Cmd->cmd_flag;
            break;

        case 8: // Arm Policy Maneuver
            a_Trg = CTRL_Cmd->cmd_val1;
            a_Rot = CTRL_Cmd->cmd_val2;
            a_Rot_bounds[0] = CTRL_Cmd->cmd_val3;
            a_Rot_bounds[1] = CTRL_Cmd->cmd_flag;

            Policy_Armed_Flag = !Policy_Armed_Flag;
            break;

        case 9: // UPDATE PLANE POSITION

            r_P_O.x = CTRL_Cmd->cmd_val1;
            r_P_O.y = CTRL_Cmd->cmd_val2;
            r_P_O.z = CTRL_Cmd->cmd_val3;
            Plane_Angle_deg = CTRL_Cmd->cmd_flag;

            updateRotationMatrices();
            
            break;

        case 10: // Upload Point-to-Point Trajectory Values

            Traj_Type = P2P;
            axis = (axis_direction)CTRL_Cmd->cmd_flag;

            switch(axis){

                case x_axis:

                    s_0_t[0] = Pos_B_O.x;           // Starting position [m]
                    s_f_t[0] = CTRL_Cmd->cmd_val2;  // Ending position [m]
                    a_t[0] = CTRL_Cmd->cmd_val3;    // Peak acceleration [m/s^2]
                    t_traj[0] = 0.0f;               // Reset timer

                    break;

                case y_axis:

                    s_0_t[1] = Pos_B_O.y;           // Starting position [m]
                    s_f_t[1] = CTRL_Cmd->cmd_val2;  // Ending position [m]
                    a_t[1] = CTRL_Cmd->cmd_val3;    // Peak acceleration [m/s^2]
                    t_traj[1] = 0.0f;               // Reset timer

                    break;

                case z_axis:

                    s_0_t[2] = Pos_B_O.z;           // Starting position [m]
                    s_f_t[2] = CTRL_Cmd->cmd_val2;  // Ending position [m]
                    a_t[2] = CTRL_Cmd->cmd_val3;    // Peak acceleration [m/s^2]
                    t_traj[2] = 0.0f;               // Reset timer

                    break;
            }

            break;

        case 11: // Upload Constant Velocity Trajectory Values

            Traj_Type = CONST_VEL;
            axis = (axis_direction)CTRL_Cmd->cmd_flag;

            switch(axis){

                case x_axis:

                    s_0_t[0] = Pos_B_O.x;           // Starting position [m]
                    v_t[0] = CTRL_Cmd->cmd_val1;    // Desired velocity [m/s]
                    a_t[0] = CTRL_Cmd->cmd_val2;    // Acceleration [m/s^2]
                    j_t[0] = CTRL_Cmd->cmd_val3;    // Jerk [m/s^3]
                    t_traj[0] = 0.0f;               // Reset timer
                    T[0] = 0.0f;                    // Reset completion time

                    break;

                case y_axis:

                    s_0_t[1] = Pos_B_O.y;
                    v_t[1] = CTRL_Cmd->cmd_val1;    
                    a_t[1] = CTRL_Cmd->cmd_val2;    
                    j_t[1] = CTRL_Cmd->cmd_val3;    
                    t_traj[1] = 0.0f;
                    T[1] = 0.0f;

                    break;

                case z_axis:

                    s_0_t[2] = Pos_B_O.z;
                    v_t[2] = CTRL_Cmd->cmd_val1;    
                    a_t[2] = CTRL_Cmd->cmd_val2;    
                    j_t[2] = CTRL_Cmd->cmd_val3;    
                    t_traj[2] = 0.0f;
                    T[2] = 0.0f;
                    
                    break;
                    
            }

            break;

        case 19: // ACTIVATE TRAJECTORY

            Traj_Active[0] = (bool)CTRL_Cmd->cmd_val1;
            Traj_Active[1] = (bool)CTRL_Cmd->cmd_val2;
            Traj_Active[2] = (bool)CTRL_Cmd->cmd_val3;

            break;
    

        case 92: // Upload Gazebo Velocity Trajectory Values (Instantaneous Acceleration)

            Traj_Type = GZ_CONST_VEL;
            axis = (axis_direction)CTRL_Cmd->cmd_flag;
            printf("Traj_Type: %d | axis: %d\n",Traj_Type,axis);

            switch(axis){

                case x_axis:

                    s_0_t[0] = Pos_B_O.x;           // Starting position [m]
                    v_t[0] = CTRL_Cmd->cmd_val2;    // Desired velocity [m/s]
                    a_t[0] = 0.0f;                  // Acceleration [m/s^2]
                    t_traj[0] = 0.0f;               // Reset timer
                    break;

                case y_axis:

                    s_0_t[1] = Pos_B_O.y;
                    v_t[1] = CTRL_Cmd->cmd_val2;
                    a_t[1] = 0.0f;
                    t_traj[1] = 0.0f;
                    break;

                case z_axis:

                    s_0_t[2] = Pos_B_O.z;
                    v_t[2] = CTRL_Cmd->cmd_val2;
                    a_t[2] = 0.0f;
                    t_traj[2] = 0.0f;
                    break;
                    
            }

            break;

        


        case 20: // Tumble-Detection
            TumbleDetect_Flag = CTRL_Cmd->cmd_flag;
            break;

        case 24: // Quad Arming
            Armed_Flag = CTRL_Cmd->cmd_flag;
            break;

        case 30: // Custom Thrust Values

            CustomThrust_Flag = true;
            thrust_override[0] = CTRL_Cmd->cmd_val1;
            thrust_override[1] = CTRL_Cmd->cmd_val2;
            thrust_override[2] = CTRL_Cmd->cmd_val3;
            thrust_override[3] = CTRL_Cmd->cmd_flag;

            break;

        case 31: // Custom M_CMD Values

            CustomMotorCMD_Flag = true;
            M_CMD_override[0] = CTRL_Cmd->cmd_val1;
            M_CMD_override[1] = CTRL_Cmd->cmd_val2;
            M_CMD_override[2] = CTRL_Cmd->cmd_val3;
            M_CMD_override[3] = CTRL_Cmd->cmd_flag;

            break;

        case 99:           

            break;

    }

}

void controlOutput(const state_t *state, const sensorData_t *sensors)
{

    // CONTROL GAINS
    Kp_p = mkvec(P_kp_xy,P_kp_xy,P_kp_z);
    Kd_p = mkvec(P_kd_xy,P_kd_xy,P_kd_z);
    Ki_p = mkvec(P_ki_xy,P_ki_xy,P_ki_z);

    Kp_R = mkvec(R_kp_xy,R_kp_xy,R_kp_z);
    Kd_R = mkvec(R_kd_xy,R_kd_xy,R_kd_z);
    Ki_R = mkvec(R_ki_xy,R_ki_xy,R_ki_z);
    
    // =========== STATE SETPOINTS =========== //
    omega_d = mkvec(0.0f,0.0f,0.0f);        // Omega-desired [rad/s]
    domega_d = mkvec(0.0f,0.0f,0.0f);       // Omega-Accl. [rad/s^2]
    quat_d = mkquat(0.0f,0.0f,0.0f,1.0f);   // Desired orientation 

    // =========== ROTATION MATRIX =========== //
    // R changes Body axes to be in terms of Global axes
    // https://www.andre-gaschler.com/rotationconverter/
    R = quat2rotmat(Quat_B_O); // Quaternion to Rotation Matrix Conversion
    b3 = mvmul(R, e_3);         // Current body vertical axis in terms of global axes | [b3 = R*e_3] 
        


    // =========== TRANSLATIONAL EFFORT =========== //
    e_x = vsub(Pos_B_O, x_d); // [e_x = pos-x_d]
    e_v = vsub(Vel_B_O, v_d); // [e_v = vel-v_d]

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
    e_w = vsub(Omega_B_O, temp1_v);        // Ang. vel error | [e_w = omega - R'*R_d*omega_d] 

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
    temp1_v = vcross(Omega_B_O, mvmul(J, Omega_B_O)); // [omega x J*omega]


    temp1_m = mmul(hat(Omega_B_O), RT_Rd); //  hat(omega)*R'*R_d
    temp2_v = mvmul(temp1_m, omega_d);      // (hat(omega)*R'*R_d)*omega_d
    temp3_v = mvmul(RT_Rd, domega_d);       // (R'*R_d*domega_d)

    temp4_v = mvmul(J, vsub(temp2_v, temp3_v)); // J*(hat(omega)*R'*R_d*omega_d - R'*R_d*domega_d)
    Gyro_dyn = vsub(temp1_v,temp4_v);

    F_thrust = vdot(F_thrust_ideal, b3);    // Project ideal thrust onto b3 vector [N]
    M = vadd(R_effort,Gyro_dyn);            // Control moments [Nm]
}

// Converts thrust in grams to their respective M_CMD values
uint16_t thrust2Motor_CMD(float f) 
{
    float a,b,c;
    float y0;
    float Motor_CMD;

    switch (SAR_Type)
    {
        case SAR_NONE:

            return 0;

        case CRAZYFLIE:

            // VOLTAGE IS WHAT DRIVES THE MOTORS, THEREFORE ADJUST M_CMD TO MEET VOLTAGE NEED
            // CALCULATE REQUIRED VOLTAGE FOR DESIRED THRUST

            if(f<=5.2f)
            {
                a = 1.28533f;
                b = 1.51239;
                c = 0.0f;
            }
            else
            {
                a = 3.23052f;
                b = -5.46911f;
                c = 5.97889f;
            }
            float voltage_needed = -b/(2*a) + sqrtf(4*a*(f-c) + b*b)/(2*a);


            // GET RATIO OF REQUIRED VOLTAGE VS SUPPLY VOLTAGE
            float supply_voltage = pmGetBatteryVoltage();
            float percentage = voltage_needed / supply_voltage;
            percentage = percentage > 1.0f ? 1.0f : percentage; // If % > 100%, then cap at 100% else keep same

            // CONVERT RATIO TO M_CMD OF MOTOR_CMD_MAX
            Motor_CMD = percentage * (float)UINT16_MAX; // Remap percentage back to M_CMD range

            // IF MINIMAL THRUST ENSURE M_CMD = 0
            if(f <= 0.25f)
            {
                Motor_CMD = 0.0f;
            }
            return Motor_CMD;
    

        case IMPULSE_MICRO:
            
            a = 600.0f;
            b = 9.93e-5f;
            c = 2.00e-1f;

            // Calculate the command threshold
            y0 = b * a * a;
            
            // Conditionally calculate the inverse
            if (f < y0) {
                Motor_CMD = sqrtf(f / b); // Use sqrtf for float
            } 
            else {
                Motor_CMD = (f - (b * a * a - c * a)) / c;
            }

            return Motor_CMD*32.0f;

        case SO_V5:

            a = 500.0f;
            b = 5.35e-4f;
            c = 7.78e-1f;

            // Calculate the command threshold
            y0 = b * a * a;
            
            // Conditionally calculate the inverse
            if (f < y0) {
                Motor_CMD = sqrtf(f / b); // Use sqrtf for float
            } 
            else {
                Motor_CMD = (f - (b * a * a - c * a)) / c;
            }

            return Motor_CMD*32.0f;

        default:
            return 0;
    }

}


bool updateOpticalFlowEst()
{
    bool UpdateOpticalFlow = false;

    #ifdef CONFIG_SAR_EXP
    // REQUEST ACCESS TO UART ARRAY
    if(xSemaphoreTake(xMutex,(TickType_t)10) == pdTRUE)
    {
        // CHECK FOR UPDATE TO ARRAY AND RETURN ACCESS
        if(isArrUpdated)
        {
            // COPY ARRAY CONTENTS
            for (int i = 0; i < UART_ARR_SIZE; i++) 
            {
                UART_arr[i] = data_arr[i];
            }
            isArrUpdated = false;
            UpdateOpticalFlow = true;
        }
        
        // RETURN ACCESS
        xSemaphoreGive(xMutex);
    }
    #endif

    #ifdef CONFIG_SAR_SIM
        // Grab data from sim camera processing plugin
        // COPY ARRAY CONTENTS
        // for (int i = 0; i < UART_ARR_SIZE; i++) 
        // {
        //     UART_arr[i] = 0;
        // }
        // isArrUpdated = false;
        UpdateOpticalFlow = true;

    #endif

    // CALC OPTICAL FLOW VALUES
    if (UpdateOpticalFlow)
    {

        // double spatial_Grad_mat[9] = {
        //     3, 1,-1,
        //     2,-2, 1,
        //     1, 1, 1,
        // };

        // double temp_Grad_vec[3] = {
        //      9,
        //     -3,
        //      7,
        // };


        // // SOLVE Ax=b EQUATION FOR OPTICAL FLOW VECTOR
        // nml_mat* A_mat = nml_mat_from(3,3,9,spatial_Grad_mat);
        // nml_mat* b_vec = nml_mat_from(3,1,3,temp_Grad_vec);
        // nml_mat_lup* LUP = nml_mat_lup_solve(A_mat);
        // nml_mat* OF_vec = nml_ls_solve(LUP,b_vec);

        // CLAMP OPTICAL FLOW VALUES
        // Theta_x_Cam = clamp(OF_vec->data[0][0],-20.0f,20.0f);
        // Theta_y_Cam = clamp(OF_vec->data[1][0],-20.0f,20.0f);
        // Tau_Cam = clamp(1/(OF_vec->data[2][0] + 1.0e-6),0.0f,5.0f);

        // Theta_x_Cam = OF_vec->data[0][0];
        // Theta_y_Cam = OF_vec->data[1][0];
        // Tau_Cam = 1/(OF_vec->data[2][0] + 1.0e-6);
        // Tau_Cam = (float)UART_arr[0];


        // nml_mat_lup_free(LUP);
        // nml_mat_free(A_mat);
        // nml_mat_free(b_vec);
        // nml_mat_free(OF_vec);


        return true;
    }
    else
    {
        return false;
    }
    

}

bool updateOpticalFlowAnalytic(const state_t *state, const sensorData_t *sensors)
{
    // TODO: ADD CAMERA OFFSETS SO THESE NUMBERS MATCH CAMERA ESTIMATION
    D_perp = Pos_P_B.z;
    if (fabsf(D_perp) < 0.02f)
    {
        D_perp = 0.0f;
    }

    struct vec r_CR_B = {0.0f,0.0f,Collision_Radius};   // {tx,ty,n_p}
    D_perp_CR = vsub(Pos_P_B,r_CR_B).z;                 // {tx,ty,n_p}
    

    // CALC OPTICAL FLOW VALUES
    Theta_x = clamp(Vel_B_P.x/D_perp,-20.0f,20.0f);
    Theta_y = clamp(Vel_B_P.y/D_perp,-20.0f,20.0f);
    Tau = clamp(D_perp/(Vel_B_P.z + 1e-6f),0.0f,5.0f);
    Tau_CR = clamp(D_perp_CR/(Vel_B_P.z + 1e-6f),-5.0f,5.0f);

    return true;
}

float firstOrderFilter(float newValue, float prevValue, float alpha) {
    return alpha * newValue + (1 - alpha) * prevValue;
}


void updateRotationMatrices()
{
    // printf("Updating Rotation Matrices\n");
    struct vec temp_a = {cosf(radians(Plane_Angle_deg)),0.0f,-sinf(radians(Plane_Angle_deg))};
    struct vec temp_b = {0.0f,1.0f,0.0f};
    struct vec temp_c = {sinf(radians(Plane_Angle_deg)),0.0f, cosf(radians(Plane_Angle_deg))};

    R_WP = mrows(temp_a,temp_b,temp_c);
    R_PW = mtranspose(R_WP);
}

