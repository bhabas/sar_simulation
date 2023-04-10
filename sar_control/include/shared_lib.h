#pragma once

#ifdef __cplusplus // If C++ compiler then compile accordingly
extern "C" {
#endif

#include "stabilizer_types.h"
#include "console.h"


extern float value_1;
extern float value_2;

struct GTC_CmdPacket{
    uint8_t cmd_type; 
    float cmd_val1;
    float cmd_val2;
    float cmd_val3;
    float cmd_flag;
    bool  cmd_rx;
} __attribute__((packed));

extern struct GTC_CmdPacket GTC_Cmd;

void controlOutput(const state_t *state, const sensorData_t *sensors);

#ifdef __cplusplus
}
#endif

