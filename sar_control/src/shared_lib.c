#include "shared_lib.h"

struct GTC_CmdPacket GTC_Cmd;

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


void GTC_Command(struct GTC_CmdPacket *GTC_Cmd)
{
    consolePrintf("Command Recieved:\n");

    switch(GTC_Cmd->cmd_type){
        case 0: // Reset
            consolePrintf("Cmd Reset: %.3f\n",(double)GTC_Cmd->cmd_val1);
            controllerOutOfTreeReset();
            break;


        case 1: // Position

            consolePrintf("Cmd Pos: %.3f\n",(double)GTC_Cmd->cmd_val1);
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

    consolePrintf("Kp_p.x: %.3f\n",(double)P_kp_xy);

    // // =========== STATE DEFINITIONS =========== //
    // statePos = mkvec(state->position.x, state->position.y, state->position.z);                      // [m]
    // stateVel = mkvec(state->velocity.x, state->velocity.y, state->velocity.z);                      // [m]
    // stateOmega = mkvec(radians(sensors->gyro.x), radians(sensors->gyro.y), radians(sensors->gyro.z));   // [rad/s]
    // stateQuat = mkquat(state->attitudeQuaternion.x,
    //                 state->attitudeQuaternion.y,
    //                 state->attitudeQuaternion.z,
    //                 state->attitudeQuaternion.w);

    // // EULER ANGLES EXPRESSED IN YZX NOTATION
    // stateEul = quat2eul(stateQuat);
    // stateEul.x = degrees(stateEul.x);
    // stateEul.y = degrees(stateEul.y);
    // stateEul.z = degrees(stateEul.z);
    
    // // =========== STATE SETPOINTS =========== //
    // omega_d = mkvec(0.0f,0.0f,0.0f);    // Omega-desired [rad/s]
    // domega_d = mkvec(0.0f,0.0f,0.0f);   // Omega-Accl. [rad/s^2]

    // eul_d = mkvec(0.0f,0.0f,0.0f);
    // quat_d = rpy2quat(eul_d);           // Desired orientation from eul angles [ZYX NOTATION]

    // // =========== ROTATION MATRIX =========== //
    // // R changes Body axes to be in terms of Global axes
    // // https://www.andre-gaschler.com/rotationconverter/
    // R = quat2rotmat(stateQuat); // Quaternion to Rotation Matrix Conversion
    // b3 = mvmul(R, e_3);         // Current body vertical axis in terms of global axes | [b3 = R*e_3] 
        


    // // =========== TRANSLATIONAL EFFORT =========== //
    // e_x = vsub(statePos, x_d); // [e_x = pos-x_d]
    // e_v = vsub(stateVel, v_d); // [e_v = vel-v_d]


    // // POS. INTEGRAL ERROR
    // e_PI.x += (e_x.x)*dt;
    // e_PI.x = clamp(e_PI.x, -i_range_xy, i_range_xy);

    // e_PI.y += (e_x.y)*dt;
    // e_PI.y = clamp(e_PI.y, -i_range_xy, i_range_xy);

    // e_PI.z += (e_x.z)*dt;
    // e_PI.z = clamp(e_PI.z, -i_range_z, i_range_z);

    // /* [F_thrust_ideal = -kp_x*e_x*(kp_x_flag) + -kd_x*e_v + -kI_x*e_PI*(kp_x_flag) + m*g*e_3 + m*a_d] */
    // temp1_v = veltmul(vneg(Kp_p), e_x);
    // temp1_v = vscl(kp_xf,temp1_v);
    // temp2_v = veltmul(vneg(Kd_p), e_v);
    // temp1_v = vscl(kd_xf,temp1_v);
    // temp3_v = veltmul(vneg(Ki_p), e_PI);
    // P_effort = vadd3(temp1_v,temp2_v,temp3_v);
    // temp1_v = vscl(m*g, e_3); // Feed-forward term
    // temp2_v = vscl(m, a_d);

    // F_thrust_ideal = vadd3(P_effort, temp1_v,temp2_v); 

    // // =========== DESIRED BODY AXES =========== // 
    // b3_d = vnormalize(F_thrust_ideal);
    // b2_d = vnormalize(vcross(b3_d, b1_d));      // [b3_d x b1_d] | body-fixed horizontal axis
    // temp1_v = vnormalize(vcross(b2_d, b3_d));
    // R_d = mcolumns(temp1_v, b2_d, b3_d);        // Desired rotation matrix from calculations

    // // =========== ROTATIONAL ERRORS =========== // 
    // RdT_R = mmul(mtranspose(R_d), R);       // [R_d'*R]
    // RT_Rd = mmul(mtranspose(R), R_d);       // [R'*R_d]

    // temp1_v = dehat(msub(RdT_R, RT_Rd));    // [dehat(R_d'*R - R'*R)]
    // e_R = vscl(0.5f, temp1_v);              // Rotation error | [eR = 0.5*dehat(R_d'*R - R'*R)]

    // temp1_v = mvmul(RT_Rd, omega_d);        // [R'*R_d*omega_d]
    // e_w = vsub(stateOmega, temp1_v);        // Ang. vel error | [e_w = omega - R'*R_d*omega_d] 

    // // ROT. INTEGRAL ERROR
    // e_RI.x += (e_R.x)*dt;
    // e_RI.x = clamp(e_RI.x, -i_range_R_xy, i_range_R_xy);

    // e_RI.y += (e_R.y)*dt;
    // e_RI.y = clamp(e_RI.y, -i_range_R_xy, i_range_R_xy);

    // e_RI.z += (e_R.z)*dt;
    // e_RI.z = clamp(e_RI.z, -i_range_R_z, i_range_R_z);

    // // =========== CONTROL EQUATIONS =========== // 
    // /* [M = -kp_R*e_R - kd_R*e_w -ki_R*e_RI + Gyro_dyn] */

    // temp1_v = veltmul(vneg(Kp_R), e_R);     // [-kp_R*e_R]
    // temp2_v = veltmul(vneg(Kd_R), e_w);     // [-kd_R*e_w]
    // temp3_v = veltmul(vneg(Ki_R), e_RI);    // [-ki_R*e_RI]
    // R_effort = vadd3(temp1_v,temp2_v,temp3_v);

    // /* Gyro_dyn = [omega x (J*omega)] - [J*( hat(omega)*R'*R_d*omega_d - R'*R_d*domega_d )] */
    // temp1_v = vcross(stateOmega, mvmul(J, stateOmega)); // [omega x J*omega]


    // temp1_m = mmul(hat(stateOmega), RT_Rd); //  hat(omega)*R'*R_d
    // temp2_v = mvmul(temp1_m, omega_d);      // (hat(omega)*R'*R_d)*omega_d
    // temp3_v = mvmul(RT_Rd, domega_d);       // (R'*R_d*domega_d)

    // temp4_v = mvmul(J, vsub(temp2_v, temp3_v)); // J*(hat(omega)*R'*R_d*omega_d - R'*R_d*domega_d)
    // Gyro_dyn = vsub(temp1_v,temp4_v);

    // F_thrust = vdot(F_thrust_ideal, b3);    // Project ideal thrust onto b3 vector [N]
    // M = vadd(R_effort,Gyro_dyn);            // Control moments [Nm]

}
