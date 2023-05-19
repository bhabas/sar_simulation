#pragma once

// STANDARD LIBRARIES
#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "shared_lib.h"

// ==================================
//         Logging Compression
// ==================================

void compressStates();
void compressSetpoints();
void compressFlipStates();
uint32_t compressXY(float x, float y); // Compresses values in range (-32.767 - 32.767)

struct {
    // Compressed positions [mm]
    uint32_t xy; 
    int16_t z;

    // Compressed velocities [mm/s]
    uint32_t vxy; 
    int16_t vz;

    // compressed quaternion, see quatcompress.h
    int32_t quat; 

    // Compressed angular velocity [milli-rad/sec]
    uint32_t wxy; 
    int16_t wz;

    // Compressed actuation states
    uint32_t Mxy;   // [N*um]
    uint32_t FMz;   // [mN | N*um]

    uint32_t MS_PWM12; 
    uint32_t MS_PWM34;

    uint32_t M_thrust12;
    uint32_t M_thrust34;

    // Compressed Optical Flow Values
    int16_t Tau;   // [milli-rad/s]
    uint32_t Theta_xy; // [milli-rad/s]
    int16_t D_perp; // [mm]

    uint32_t NN_FP; // NN_flip,NN_policy

} StatesZ_GTC;


struct {
    
    uint32_t xy;  // Compressed position [mm]
    int16_t z;

    uint32_t vxy; // Compressed velocities [mm/s]
    int16_t vz;

    uint32_t axy; // Compress accelerations [mm/s^2]
    int16_t az;

} setpointZ_GTC;


struct {

    // Compressed positions [mm]
    uint32_t xy; 
    int16_t z;

    // Compressed velocities [mm/s]
    uint32_t vxy; 
    int16_t vz;

    // compressed quaternion, see quatcompress.h
    int32_t quat; 

    // Compressed angular velocity [milli-rad/sec]
    uint32_t wxy; 
    int16_t wz;

    // Compressed Optical Flow Values
    int16_t Tau;   // [milli-rad/s]
    uint32_t Theta_xy; // [milli-rad/s]
    int16_t D_perp; // [m]

    uint32_t NN_FP; // NN_flip,NN_policy

} FlipStatesZ_GTC;


