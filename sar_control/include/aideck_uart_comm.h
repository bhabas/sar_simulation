#pragma once

#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#define UART_ARR_SIZE 16
extern int32_t data_arr[UART_ARR_SIZE];  // Array to store received values
extern bool isArrUpdated;