#ifdef __cplusplus // If C++ compiler then compile accordingly
extern "C" {
#endif

#include "NN_Layers_Policy_WL.h"
#include "NN_Layers_Flip_WL.h"

// STANDARD LIBRARIES
#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>

// CF LIBARARIES
#include "math3d.h"
// #include "log.h"
// #include "param.h"
// #include "debug.h"
// #include "motors.h"

// CF HEADERS
#include "stabilizer_types.h"
// #include "physicalConstants.h"
// #include "quatcompress.h"
#include "nml.h"

void controllerGTC(control_t *control, setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick);



























#ifdef __cplusplus
}
#endif