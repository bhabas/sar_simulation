
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <time.h>
#include <float.h>
#include <math.h>
#include <string.h>


#define CML_MIN_COEF 0.000000000000001

typedef struct{
    int num_rows;
    int num_cols;
    float data[3][3];
}cml_m33;

typedef struct{
    int num_rows;
    int num_cols;
    float data[3][1];
}cml_m31;


void cml_mat_print(void* m, int rows, int cols);

cml_m33 cml_m33_new();
cml_m31 cml_m31_new();

#ifdef __cplusplus
}
#endif