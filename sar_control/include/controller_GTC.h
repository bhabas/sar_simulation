#ifdef __cplusplus // If C++ compiler then compile accordingly
extern "C" {
#endif

#ifdef CONFIG_SAR_SIM
#define consolePrintf printf
#define DEBUG_PRINT printf
#endif


// STANDARD LIBRARIES
#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>

// CF LIBARARIES
#include "app.h"
#include "stabilizer_types.h"


#ifdef CONFIG_SAR_EXP
#include "FreeRTOS.h"
#include "task.h"
#include "console.h"
#endif


#ifdef __cplusplus
}
#endif
