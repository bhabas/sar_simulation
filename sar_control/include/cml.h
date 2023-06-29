
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
    float data[3][3];
    int num_rows;
    int num_cols;
}cml_m33;

typedef struct{
    float data[3][1];
    int num_rows;
    int num_cols;
}cml_m31;



void cml_m33_init(cml_m33* m);
void cml_m31_init(cml_m31* m);

void cml_mat_print(void* m, int m_rows, int m_cols);
void cml_mat_fill_fromarr(void* m, int m_rows, int m_cols, float arr[], int arr_size);
cml_m31 cml_m31_col_get(cml_m33* m, int col);
float cml_vec_dot(cml_m33* m1, int m1_col, cml_m33* m2, int m2_col);



#ifdef __cplusplus
}
#endif