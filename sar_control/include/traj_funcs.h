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

extern struct traj_vec Traj_Activate;
extern struct traj_vec s_0_t;           // Traj Start Point [m]
extern struct traj_vec s_f_t;           // Traj End Point [m]
extern struct traj_vec v_t;             // Traj Vel [m/s]
extern struct traj_vec a_t;             // Traj Accel [m/s^2]
extern struct traj_vec T;               // Traj completion time [s]
extern struct traj_vec t_traj;          // Traj time counter [s]

static void set_vec_element(struct vec *v, int index, float value) {

    switch (index) {
        case 0:
            v->x = value;
            break;
        case 1:
            v->y = value;
            break;
        case 2:
            v->z = value;
            break;
        default:
            // Handle invalid index if necessary
            break;
    }
}

void point2point_Traj();




#ifdef __cplusplus
}
#endif
