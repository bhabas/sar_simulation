#ifdef __cplusplus // If C++ compiler then compile accordingly
extern "C" {
#endif


#include <stdio.h>
#include "stabilizer_types.h"

void controllerGTC(control_t *control, setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick);



























#ifdef __cplusplus
}
#endif