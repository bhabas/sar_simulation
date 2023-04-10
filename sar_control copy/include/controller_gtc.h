#ifdef __cplusplus // If C++ compiler then compile accordingly
extern "C" {
#endif

#define consolePrintf printf
#define DEBUG_PRINT printf


// CRAZYFLIE FIRMWARE CODE BELOW
// ===========================================================


/**
 *
 * controller_gtc.h - Geometric Tracking Controller Interface
 *
 */

#ifndef __CONTROLLER_GTC_H__
#define __CONTROLLER_GTC_H__

#include "ML_Params/NN_Layers_NL_DR.h"
#include "ML_Params/SVM_Params_NL_DR.h"
#include "ML_Params/NN_Layers_NL_DeepRL.h"

// STANDARD LIBRARIES
#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>

// CF LIBARARIES
// #include "math3d.h"
#include "log.h"
#include "param.h"
#include "debug.h"
#include "motors.h"
#include "pm.h"

// CF HEADERS
#include "stabilizer_types.h"
#include "physicalConstants.h"
#include "quatcompress.h"
#include "nml.h"


#include "shared_lib.h"

#define PWM_MAX 60000
#define f_MAX 15.0f 
#define g2Newton (9.81f/1000.0f)
#define Newton2g (1000.0f/9.81f)


// FUNCTION PRIMITIVES
void controllerOutOfTreeInit();
bool controllerOutOfTreeTest();
void controllerOutOfTreeReset(void);
void controllerOutOfTree(control_t *control,const setpoint_t *setpoint, 
                                            const sensorData_t *sensors, 
                                            const state_t *state, 
                                            const uint32_t tick) ;
void appMain();
void controllerGTCTraj(void);
void calcOpticalFlow(const state_t* state, const sensorData_t* sensors);
void velocity_Traj(void);
void GZ_velocity_Traj(void);
void point2point_Traj(void);






// Converts thrust in grams to their respective PWM values
static int32_t thrust2PWM(float f) 
{
    // VOLTAGE IS WHAT DRIVES THE MOTORS, THEREFORE ADJUST PWM TO MEET VOLTAGE NEED

    // CALCULATE REQUIRED VOLTAGE FOR DESIRED THRUST

    float a,b,c;

    if(f<=5.2f)
    {
        a = 1.28533f;
        b = 1.51239;
        c = 0.0f;
    }
    else
    {
        a = 3.23052f;
        b = -5.46911f;
        c = 5.97889f;
    }
    


    float voltage_needed = -b/(2*a) + sqrtf(4*a*(f-c) + b*b)/(2*a);


    // GET RATIO OF REQUIRED VOLTAGE VS SUPPLY VOLTAGE
    float supply_voltage = pmGetBatteryVoltage();
    float percentage = voltage_needed / supply_voltage;
    percentage = percentage > 1.0f ? 1.0f : percentage; // If % > 100%, then cap at 100% else keep same

    // CONVERT RATIO TO PWM OF PWM_MAX
    float PWM = percentage * (float)UINT16_MAX; // Remap percentage back to PWM range

    // IF MINIMAL THRUST ENSURE PWM = 0
    if(f <= 0.25f)
    {
        PWM = 0.0f;
    }

    return PWM;

}      


#endif //__CONTROLLER_GTC_H__


// ===========================================================








#ifdef __cplusplus
}
#endif