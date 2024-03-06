#include "Compress_States.h"

// Definition of the global instances
States_Z_Struct States_Z;
TrgStates_Z_Struct TrgStates_Z;
SetPoints_Z_Struct SetPoints_Z;

void compressStates(){

    // COMPRESS FULL STATE VALUES
    States_Z.r_BOxy = compressXY(Pos_B_O.x,Pos_B_O.y);
    States_Z.r_BOz = (int16_t)(Pos_B_O.z * 1000.0f);

    States_Z.V_BOxy = compressXY(Vel_B_O.x, Vel_B_O.y);
    States_Z.V_BOz = (int16_t)(Vel_B_O.z * 1000.0f);

    States_Z.Acc_BOxy = compressXY(Accel_B_O.x, Accel_B_O.y);
    States_Z.Acc_BOz = (int16_t)(Accel_B_O.z * 1000.0f);

    float const q[4] = {
        Quat_B_O.x,
        Quat_B_O.y,
        Quat_B_O.z,
        Quat_B_O.w};
    States_Z.Quat_BO = quatcompress(q);

    States_Z.Omega_BOxy = compressXY(Omega_B_O.x/10,Omega_B_O.y/10);
    States_Z.Omega_BOz = (int16_t)(Omega_B_O.z * 1000.0f);

    States_Z.dOmega_BOy = (int16_t)(dOmega_B_O.y * 1000.0f);

    States_Z.VelRel_BP = compressXY(Vel_mag_B_P, Vel_angle_B_P);

    States_Z.D_perp = compressXY(D_perp,D_perp_CR);
    States_Z.Tau = compressXY(Tau,Tau_CR);
    States_Z.Theta_x = (int16_t)(Theta_x * 1000.0f);

    States_Z.r_PBxy = compressXY(Pos_P_B.x,Pos_P_B.y);
    States_Z.r_PBz = (int16_t)(Pos_P_B.z * 1000.0f);


    // COMPRESS THRUST/MOMENT VALUES
    States_Z.FMz = compressXY(F_thrust,M.z*1000.0f);    // [N, N*mm]
    States_Z.Mxy = compressXY(M.x*1000.0f,M.y*1000.0f); // [N*mm, N*mm]

    // COMPRESS MOTOR THRUST VALUES
    States_Z.M_thrust12 = compressXY(M1_thrust*1.0e-2f,M2_thrust*1.0e-2f);
    States_Z.M_thrust34 = compressXY(M3_thrust*1.0e-2f,M4_thrust*1.0e-2f);

    // COMPRESS MS CMD VALUES
    States_Z.M_CMD12 = compressXY(M1_CMD*0.5e-3f,M2_CMD*0.5e-3f);
    States_Z.M_CMD34 = compressXY(M3_CMD*0.5e-3f,M4_CMD*0.5e-3f);


    // COMPRESS POLICY ACTIONS
    States_Z.Policy_Actions = compressXY(a_Trg,a_Rot);
}

void compressTrgStates(){

    // COMPRESS FULL STATE VALUES
    TrgStates_Z.r_BOxy = compressXY(Pos_B_O_trg.x,Pos_B_O_trg.y);
    TrgStates_Z.r_BOz = (int16_t)(Pos_B_O_trg.z * 1000.0f);

    TrgStates_Z.V_BOxy = compressXY(Vel_B_O_trg.x, Vel_B_O_trg.y);
    TrgStates_Z.V_BOz = (int16_t)(Vel_B_O_trg.z * 1000.0f);

    float const q[4] = {
        Quat_B_O_trg.x,
        Quat_B_O_trg.y,
        Quat_B_O_trg.z,
        Quat_B_O_trg.w};
    TrgStates_Z.Quat_BO = quatcompress(q);

    TrgStates_Z.Omega_BOy = (int16_t)(Omega_B_O_trg.y * 1000.0f);

    TrgStates_Z.VelRel_BP = compressXY(Vel_mag_B_P_trg, Vel_angle_B_P_trg);

    TrgStates_Z.D_perp = compressXY(D_perp_trg,D_perp_CR_trg);
    TrgStates_Z.Tau = compressXY(Tau_trg,Tau_CR_trg);
    TrgStates_Z.Theta_x = (int16_t)(Theta_x_trg * 1000.0f);

    TrgStates_Z.r_PBxy = compressXY(Pos_P_B_trg.x,Pos_P_B_trg.y);
    TrgStates_Z.r_PBz = (int16_t)(Pos_P_B_trg.z * 1000.0f);


    // COMPRESS POLICY ACTIONS
    TrgStates_Z.Policy_Actions = compressXY(a_Trg_trg,a_Rot_trg);

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
