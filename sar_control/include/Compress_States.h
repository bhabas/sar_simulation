#pragma once

// STANDARD LIBRARIES
#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "Shared_Lib.h"

// ==================================
//         Logging Compression
// ==================================

void compressStates();
void compressTrgStates();
void compressImpactOBStates();
void compressSetpoints();
uint32_t compressXY(float x, float y); // Compresses values in range (-32.767 - 32.767)

typedef struct {
    // Compressed positions [mm]
    uint32_t r_BOxy; 
    int16_t r_BOz;

    // Compressed velocities [mm/s]
    uint32_t V_BOxy; 
    int16_t V_BOz;

    // Compressed accelerations [mm/s^2]
    uint32_t Acc_BOxy; 
    int16_t Acc_BOz;
    int16_t Accel_BO_Mag;

    // compressed quaternion, see quatcompress.h
    int32_t Quat_BO; 

    // Compressed angular velocity [milli-rad/sec]
    uint32_t Omega_BOxy; 
    int16_t Omega_BOz;

    // Compressed angular acceleration [milli-rad/sec^2]
    int16_t dOmega_BOy;

    // Compressed Relative Position [mm]
    uint32_t r_PBxy;
    int16_t r_PBz;

    // Compressed Relative Velocity [mm/s]
    uint32_t VelRel_BP;

    // Compressed Distance to Surface [mm]
    uint32_t D_perp;

    // Compressed Optical Flow Cues
    uint32_t Tau;
    int16_t Theta_x;


    // Compressed actuation states
    uint32_t Mxy;   // [N*um]Values
    uint32_t FMz;   // [mN | N*um]

    uint32_t M_CMD12; 
    uint32_t M_CMD34;

    uint32_t M_thrust12;    // [mg]
    uint32_t M_thrust34;    // [mg]


    // Compressed Policy Actions
    uint32_t Policy_Actions;


} States_Z_Struct;

typedef struct {

    // Compressed positions [mm]
    uint32_t r_BOxy; 
    int16_t r_BOz;

    // Compressed velocities [mm/s]
    uint32_t V_BOxy; 
    int16_t V_BOz;

    // Compressed accelerations [mm/s^2]
    uint32_t Acc_BOxy; 
    int16_t Acc_BOz;

    // compressed quaternion, see quatcompress.h
    int32_t Quat_BO; 

    // Compressed angular velocity [milli-rad/sec]
    int16_t Omega_BOy;

    // Compressed angular acceleration [milli-rad/sec^2]
    int16_t dOmega_BOy;

    // Compressed Relative Position [mm]
    uint32_t r_PBxy;
    int16_t r_PBz;

    // Compressed Relative Velocity [mm/s]
    uint32_t VelRel_BP;

    // Compressed Distance to Surface [mm]
    uint32_t D_perp;

    // Compressed Optical Flow Cues
    uint32_t Tau;
    int16_t Theta_x;

    // Compressed Policy Actions
    uint32_t Policy_Actions;
   

} TrgStates_Z_Struct;

typedef struct {

    // Compressed Velocity [mm/s]
    uint32_t VelRel_BP;

    // compressed quaternion, see quatcompress.h
    int32_t Quat_BO; 

    // Compressed angular velocity [milli-rad/sec]
    int16_t Omega_BOy;

    // Compressed Impact angular acceleration [milli-rad/sec^2]
    int16_t dOmega_BOy;
   

} ImpactOB_States_Z_Struct;

typedef struct {
    
    uint32_t xy;  // Compressed position [mm]
    int16_t z;

    uint32_t vxy; // Compressed velocities [mm/s]
    int16_t vz;

    uint32_t axy; // Compress accelerations [mm/s^2]
    int16_t az;

} SetPoints_Z_Struct;

// Declaration of the global instances
extern States_Z_Struct States_Z;
extern TrgStates_Z_Struct TrgStates_Z;
extern ImpactOB_States_Z_Struct ImpactOB_States_Z;
extern SetPoints_Z_Struct SetPoints_Z;
