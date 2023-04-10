#include "shared_lib.h"

float value_1 = 5.4f;
float value_2 = 3.14f;

struct GTC_CmdPacket GTC_Cmd;


void GTC_Command(struct GTC_CmdPacket *GTC_Cmd)
{
    consolePrintf("GTC RX value1: %.3f\n",(double)GTC_Cmd->cmd_val1);

}

void controlOutput(const state_t *state, const sensorData_t *sensors)
{

    consolePrintf("controlOutput State: %.3f\n",(double)state->position.x);

}