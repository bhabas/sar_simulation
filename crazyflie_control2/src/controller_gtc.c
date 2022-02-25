// CF HEADERS
#include "controller_gtc.h"



void controllerGTCInit(void)
{
  
    consolePrintf("GTC Initiated\n");
}

void controllerGTCReset(void)
{
    consolePrintf("GTC Reset\n");

}

bool controllerGTCTest(void)
{
    return true;
}

void GTC_Command(setpoint_t *setpoint)
{   
    
    
}

void controllerGTCTraj()
{
   

    
}


int val = 0;
void controllerGTC(control_t *control, setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick)
{
    if (RATE_DO_EXECUTE(5, tick)) {

        val++;
    }
    

}
