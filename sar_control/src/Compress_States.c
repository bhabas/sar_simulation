#include "Compress_States.h"

// Definition of the global instances
States_Z_Struct States_Z;
TrgStates_Z_Struct TrgStates_Z;
SetPoints_Z_Struct SetPoints_Z;

void compressStates(){

    // COMPRESS FULL STATE VALUES
    States_Z.xy = compressXY(statePos.x,statePos.y);
    States_Z.z = (int16_t)(statePos.z * 1000.0f);

    States_Z.D_perp = (int16_t)(D_perp * 1000.0f);

    States_Z.vxy = compressXY(stateVel.x, stateVel.y);
    States_Z.vz = (int16_t)(stateVel.z * 1000.0f);

    States_Z.wxy = compressXY(stateOmega.x/10,stateOmega.y/10);
    States_Z.wz = (int16_t)(stateOmega.z * 1000.0f);

    float const q[4] = {
        stateQuat.x,
        stateQuat.y,
        stateQuat.z,
        stateQuat.w};
    States_Z.quat = quatcompress(q);

    // COMPRESS THRUST/MOMENT VALUES
    States_Z.FMz = compressXY(F_thrust,M.z*1000.0f);
    States_Z.Mxy = compressXY(M.x*1000.0f,M.y*1000.0f);

    // COMPRESS MOTOR THRUST VALUES
    States_Z.M_thrust12 = compressXY(M1_thrust,M2_thrust);
    States_Z.M_thrust34 = compressXY(M3_thrust,M4_thrust);

    // COMPRESS PWM VALUES
    States_Z.MS_PWM12 = compressXY(M1_pwm*0.5e-3f,M2_pwm*0.5e-3f);
    States_Z.MS_PWM34 = compressXY(M3_pwm*0.5e-3f,M4_pwm*0.5e-3f);

    // COMPRESS OPTICAL FLOW VALUES
    States_Z.Theta_xy = compressXY(Theta_x,Theta_y);
    States_Z.Tau = (int16_t)(Tau * 1000.0f); 

    // COMPRESS OPTICAL FLOW ESTIMATES
    States_Z.Theta_xy_est = compressXY(Theta_x_est,Theta_y_est);
    States_Z.Tau_est = (int16_t)(Tau_est * 1000.0f); 

    // COMPRESS POLICY ACTIONS
    States_Z.Policy_Actions = compressXY(Policy_Trg_Action,Policy_Flip_Action);
}

void compressTrgStates(){

    // COMPRESS FULL STATE VALUES
    TrgStates_Z.xy = compressXY(statePos_trg.x,statePos_trg.y);
    TrgStates_Z.z = (int16_t)(statePos_trg.z * 1000.0f);

    TrgStates_Z.D_perp = (int16_t)(D_perp_trg * 1000.0f);

    TrgStates_Z.vxy = compressXY(stateVel_trg.x, stateVel_trg.y);
    TrgStates_Z.vz = (int16_t)(stateVel_trg.z * 1000.0f);

    TrgStates_Z.wxy = compressXY(stateOmega_trg.x,stateOmega_trg.y);
    TrgStates_Z.wz = (int16_t)(stateOmega_trg.z * 1000.0f);

    float const q[4] = {
        stateQuat_trg.x,
        stateQuat_trg.y,
        stateQuat_trg.z,
        stateQuat_trg.w};
    TrgStates_Z.quat = quatcompress(q);

    // COMPRESS OPTICAL FLOW VALUES
    TrgStates_Z.Theta_xy = compressXY(Theta_x_trg,Theta_y_trg);
    TrgStates_Z.Tau = (int16_t)(Tau_trg * 1000.0f); 

    // COMPRESS OPTICAL FLOW ESTIMATES
    TrgStates_Z.Theta_xy_est = compressXY(Theta_x_est_trg,Theta_y_est_trg);
    TrgStates_Z.Tau_est = (int16_t)(Tau_est_trg * 1000.0f); 

    // COMPRESS POLICY ACTIONS
    TrgStates_Z.Policy_Actions = compressXY(Policy_Trg_Action_trg,Policy_Flip_Action_trg);

}

void compressSetpoints(){
    SetPoints_Z.xy = compressXY(x_d.x,x_d.y);
    SetPoints_Z.z = (int16_t)(x_d.z * 1000.0f);

    SetPoints_Z.vxy = compressXY(v_d.x,v_d.y);
    SetPoints_Z.vz = (int16_t)(v_d.z * 1000.0f);

    SetPoints_Z.axy = compressXY(a_d.x,a_d.y);
    SetPoints_Z.az = (int16_t)(a_d.z * 1000.0f);
}



uint32_t compressXY(float x, float y)
{ 
  
  uint16_t xnew, ynew;
  uint32_t xy;

  // CONVERT FLOATS TO INTS OFFSET BY UINT16_MAX/2.0
  xnew = x*1000.0f + 32767.0f;
  ynew = y*1000.0f + 32767.0f;


  // CLIP RANGES OF VALUES
  xnew = (xnew < UINT16_MAX) ? xnew : UINT16_MAX;
  xnew = (xnew > 0) ? xnew : 0;

  ynew = (ynew < UINT16_MAX) ? ynew : UINT16_MAX;
  ynew = (ynew > 0) ? ynew : 0;

  // APPEND YNEW BYTES TO XNEW BYTES
  xy = (xnew << 16 | ynew); // Shift xnew by 16 and combine

  return xy;
};
