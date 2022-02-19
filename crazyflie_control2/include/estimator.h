#include <stdio.h>
#include "stabilizer_types.h"


void stateEstimator(state_t *state, sensorData_t *sensors, control_t *control, const uint32_t tick);