#pragma once

#ifdef __cplusplus // If C++ compiler then compile accordingly
extern "C" {
#endif

#include "stabilizer_types.h"
#include "console.h"


extern float value_1;
extern float value_2;

void controlOutput(const state_t *state, const sensorData_t *sensors);

#ifdef __cplusplus
}
#endif

