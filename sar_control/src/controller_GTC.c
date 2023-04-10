#include "controller_GTC.h"


void appMain() {

    printf("Hello app\n");

  
}


void controllerOutOfTreeInit() {
    printf("GTC init\n");

}

bool controllerOutOfTreeTest() {

  return true;
}

void controllerOutOfTree(control_t *control,const setpoint_t *setpoint, 
                                            const sensorData_t *sensors, 
                                            const state_t *state, 
                                            const uint32_t tick) 
{

    printf("GTC loop\n");
  

}