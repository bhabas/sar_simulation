#include "controller_GTC.h"



void appMain() {

    while (1)
    {
        #ifdef CONFIG_SAR_SIM
        // consolePrintf("Hello app\n");
        #elif CONFIG_SAR_EXP
        consolePrintf("Hello app\n");
        vTaskDelay(M2T(2000));
        #endif
    }
    
}


void controllerOutOfTreeInit() {
    consolePrintf("GTC init\n");

    ledSet(LED_BLUE_L, 0);
    ledSet(LED_BLUE_NRF, 0);

}

bool controllerOutOfTreeTest() {

  return true;
}

void controllerOutOfTree(control_t *control,const setpoint_t *setpoint, 
                                            const sensorData_t *sensors, 
                                            const state_t *state, 
                                            const uint32_t tick) 
{

    if (RATE_DO_EXECUTE(2, tick))
    {
        consolePrintf("GTC loop value1: %.3f\n",(double)value_1);
    }
 

}