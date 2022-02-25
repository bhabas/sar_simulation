#ifdef __cplusplus // If C++ compiler then compile accordingly
extern "C" {
#endif


// CRAZYFLIE FIRMWARE CODE BELOW
// ===========================================================


/**
 *
 * controller_gtc.h - Geometric Tracking Controller Interface
 *
 */

#ifndef __CONTROLLER_GTC_H__
#define __CONTROLLER_GTC_H__

#include "NN_Params/NN_Layers_Policy_WL.h"
#include "NN_Params/NN_Layers_Flip_WL.h"

// STANDARD LIBRARIES
#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>

// CF LIBARARIES
#include "math3d.h"
#include "log.h"
#include "param.h"
#include "debug.h"
#include "motors.h"

// CF HEADERS
#include "stabilizer_types.h"
#include "physicalConstants.h"
#include "quatcompress.h"
#include "nml.h"

#define PWM_MAX 60000
#define f_MAX (16.5)
#define g2Newton (9.81f/1000.0f)
#define Newton2g (1000.0f/9.81f)

typedef struct{
    nml_mat* mean;
    nml_mat* std;
}Scaler;

// FUNCTION PRIMITIVES
void controllerGTCInit(void);
bool controllerGTCTest(void);
void controllerGTCReset(void);
void controllerGTCTraj(void);
void controllerGTC(control_t *control, setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick);
void GTC_Command(setpoint_t *setpoint);





extern int val;

#endif //__CONTROLLER_GTC_H__


// ===========================================================






#define consolePrintf printf


#ifdef __cplusplus
}
#endif