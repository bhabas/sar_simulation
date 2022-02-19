#include "estimator.h"


void stateEstimator(state_t *state, sensorData_t *sensors, control_t *control, const uint32_t tick)
{
    printf("X: %.3f\n",state->position.x);

}