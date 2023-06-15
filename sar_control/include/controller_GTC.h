#pragma once

#ifdef __cplusplus // If C++ compiler then compile accordingly
extern "C" {
#endif

// STANDARD LIBRARIES
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>

// CF LIBARARIES
#include "app.h"
#include "stabilizer_types.h"
#include "controller.h"
#include "console.h"
#include "math3d.h"

// CUSTOM LIBRARIES
#include "shared_lib.h"
#include "nml.h"

#ifdef CONFIG_SAR_EXP
#include "FreeRTOS.h"
#include "task.h"
#include "app_channel.h"
#include "log.h"
#include "param.h"
#include "motors.h"
#include "led.h"
#endif


void controllerOutOfTreeReset();


#ifdef __cplusplus
}
#endif
