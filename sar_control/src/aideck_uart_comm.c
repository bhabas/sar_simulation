#include "aideck_uart_comm.h"

// INIT DATA ARRAY
int32_t data_arr[UART_ARR_SIZE];
int data_counter = 0;
uint8_t buffer[4];
bool isArrUpdated = false;