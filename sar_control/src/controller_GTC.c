#include "controller_GTC.h"



void appMain() {

    while (1)
    {
        #ifdef CONFIG_SAR_SIM
        consolePrintf("Hello app\n");
        #elif CONFIG_SAR_EXP
        consolePrintf("Hello app\n");
        vTaskDelay(M2T(2000));
        #endif
    }
    
}


void controllerOutOfTreeInit() {
    consolePrintf("GTC init\n");

}

bool controllerOutOfTreeTest() {

  return true;
}

void controllerOutOfTree(control_t *control,const setpoint_t *setpoint, 
                                            const sensorData_t *sensors, 
                                            const state_t *state, 
                                            const uint32_t tick) 
{

    // consolePrintf("GTC loop\n");
  

}