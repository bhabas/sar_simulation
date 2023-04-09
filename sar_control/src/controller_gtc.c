// CF HEADERS
#include "controller_gtc.h"
#include "ML_funcs.h"
#include "CompressedStates.h"
#include "led.h"



void appMain() {

    while(1) {

        #ifdef GAZEBO_SIM
        // EXECUTE GTC COMMAND WHEN RECEIVED
        if (GTC_Cmd.cmd_rx == true)
        {
            GTC_Command(&GTC_Cmd);
        }
        #else
        // WAIT UNTIL GTC COMMAND IS RECEIVED
        if (appchannelReceiveDataPacket(&GTC_Cmd,sizeof(GTC_Cmd),APPCHANNEL_WAIT_FOREVER))
        {
            if (GTC_Cmd.cmd_rx == true) GTC_Command(&GTC_Cmd);
        }
        #endif
    }
  
}

void GTC_Command(struct GTC_CmdPacket *GTC_Cmd)
{
    // MARK THAT COMMAND IS BEING ENACTED
    GTC_Cmd->cmd_rx = false;

    consolePrintf("Command Recieved: %.3f\n",GTC_Cmd->cmd_val1);
    switch(GTC_Cmd->cmd_type){
        case 0: // Reset
            controllerOutOfTreeReset();
            break;


        case 1: // Position
            x_d.x = GTC_Cmd->cmd_val1;
            x_d.y = GTC_Cmd->cmd_val2;
            x_d.z = GTC_Cmd->cmd_val3;
            kp_xf = GTC_Cmd->cmd_flag;
            break;


        case 2: // Velocity
            v_d.x = GTC_Cmd->cmd_val1;
            v_d.y = GTC_Cmd->cmd_val2;
            v_d.z = GTC_Cmd->cmd_val3;
            kd_xf = GTC_Cmd->cmd_flag;
            break;


        case 4: // Euler Angle

            // TODO: ADD ANGLE SETPOINT OPTION INTO CONTROLLER FOR ANGLE BASED POLICY

            break;        

        case 5: // Hard Set All Motorspeeds to Zero
            motorstop_flag = true;
            break;


    }
    

}

void controllerOutOfTreeInit(void)
{
    // TURN OFF BLUE LEDS (THEY'RE VERY BRIGHT)
    ledSet(LED_BLUE_L, 0);
    ledSet(LED_BLUE_NRF, 0);

    controllerOutOfTreeReset();
    controllerOutOfTreeTest();
    consolePrintf("GTC Initiated\n");
}

void controllerOutOfTreeReset(void)
{
    consolePrintf("GTC Reset\n");
    // consolePrintf("Policy_Type: %d\n",Policy);

    // RESET ERRORS
    e_PI = vzero();
    e_RI = vzero();

    // TURN POS/VEL CONTROLLER FLAGS ON
    kp_xf = 1.0f;
    kd_xf = 1.0f;

    // RESET SETPOINTS TO HOME POSITION
    x_d = mkvec(0.0f,0.0f,0.4f);
    v_d = mkvec(0.0f,0.0f,0.0f);
    a_d = mkvec(0.0f,0.0f,0.0f);
    b1_d = mkvec(1.0f,0.0f,0.0f);

    // RESET SYSTEM FLAGS
    tumbled = false;
    motorstop_flag = false;

}

bool controllerOutOfTreeTest(void)
{   
    return true;
}


void controllerOutOfTree(control_t *control,const setpoint_t *setpoint, 
                                            const sensorData_t *sensors, 
                                            const state_t *state, 
                                            const uint32_t tick) 
{
    // UPDATE OPTICAL FLOW VALUES AT 100 HZ
    if (RATE_DO_EXECUTE(RATE_100_HZ, tick)) {

    }
    

    if (RATE_DO_EXECUTE(RATE_500_HZ, tick)) {


        controlOutput(state,sensors);

    }

}


#ifndef GAZEBO_SIM
PARAM_GROUP_START(my_PARAM)
    PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, value_1, &value_1)
PARAM_GROUP_STOP(my_PARAM)

LOG_GROUP_START(my_LOG)
    LOG_ADD(LOG_FLOAT, value_2, &value_2)
LOG_GROUP_STOP(my_LOG)
#endif
