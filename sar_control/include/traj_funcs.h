#pragma once

#ifdef __cplusplus // If C++ compiler then compile accordingly
extern "C" {
#endif

// STANDARD LIBRARIES
#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>

// CF LIBARARIES
#include "math3d.h"



// =================================
//     TRAJECTORY INITIALIZATION
// =================================

typedef enum {
    NONE = 0,
    P2P = 1,
    CONST_VEL = 2,
}Trajectory_Type;
extern Trajectory_Type Traj_Type;

typedef enum {
    x_axis = 0, 
    y_axis = 1,
    z_axis = 2
} axis_direction;
extern axis_direction axis;

#ifdef __cplusplus
}
#endif
