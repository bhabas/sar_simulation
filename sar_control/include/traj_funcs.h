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
#include "shared_lib.h"



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

struct traj_vec{
  union {
    struct {
      float x;
      float y;
      float z;
    };
    float idx[3];
  };
};

extern float Traj_Activate[3];
extern float s_0_t[3];              // Traj Start Point [m]
extern float s_f_t[3];              // Traj End Point [m]
extern float v_t[3];                // Traj Vel [m/s]
extern float a_t[3];                // Traj Accel [m/s^2]
extern float T[3];                  // Traj completion time [s]
extern float t_traj[3];             // Traj time counter [s]


void set_vec_element(struct vec *v, int index, float value);
void point2point_Traj();




#ifdef __cplusplus
}
#endif
