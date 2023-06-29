
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

typedef struct{
  cml_m33 *L;
  cml_m33 *U;
  cml_m33 *P;
  int num_permutations;
} cml_m33_lup;


void cml_m33_init(cml_m33* m);
void cml_m31_init(cml_m31* m);

void cml_mat_fill_fromarr(void* m, int m_rows, int m_cols, float arr[], int arr_size);
cml_m33 cml_m33_eye();
int cml_mat_diag_set(cml_m33 *m, double value);


void cml_m33_row_swap_r(cml_m33* m, int row1, int row2);
void cml_m33_col_swap_r(cml_m33* m, int col1, int col2);
void cml_m33_row_addrow_r(cml_m33* m, int where, int row, float multiplier);
int cml_m33_absmaxr(cml_m33* m, int k);


void cml_mat_print(void* m, int m_rows, int m_cols);
cml_m31 cml_m31_col_get(cml_m33* m, int col);
float cml_vec_dot(cml_m33* m1, int m1_col, cml_m33* m2, int m2_col);

cml_m33_lup *cml_mat_lup_init(cml_m33 *L, cml_m33 *U, cml_m33 *P, int num_permutations);



#ifdef __cplusplus
}
#endif