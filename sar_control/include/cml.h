
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
  cml_m33 L;
  cml_m33 U;
  cml_m33 P;
  int num_permutations;
} cml_m33_lup;

void cml_m31_init(cml_m31* m);
cml_m31 cml_m31_new();

void cml_m33_init(cml_m33* m);
cml_m33 cml_m33_new();
cml_m33 cml_m33_eye();
cml_m33 cml_m33_cp(cml_m33* m);

void cml_mat_fill_fromarr(void* m, int m_rows, int m_cols, float arr[], int arr_size);
int cml_mat_diag_set(cml_m33 *m, double value);


void cml_m33_row_swap_r(cml_m33* m, int row1, int row2);
void cml_m33_col_swap_r(cml_m33* m, int col1, int col2);
void cml_m33_row_addrow_r(cml_m33* m, int where, int row, float multiplier);
int cml_m33_absmaxr(cml_m33* m, int k);


void cml_mat_print(void* m, int m_rows, int m_cols);
cml_m31 cml_m31_col_get(cml_m33* m, int col);
float cml_vec_dot(cml_m33* m1, int m1_col, cml_m33* m2, int m2_col);

cml_m33_lup cml_m33_lup_new(cml_m33 L, cml_m33 U, cml_m33 P, int num_permutations);
void cml_m33_lup_print(cml_m33_lup *lup);
cml_m33_lup cml_m33_lup_solve(cml_m33* m);

cml_m31 cml_ls_solvefwd(cml_m33 *L, cml_m31 *b);
cml_m31 cml_ls_solvebck(cml_m33 *U, cml_m31 *b);
cml_m31 cml_ls_solve(cml_m33_lup *lu, cml_m31* b);





#ifdef __cplusplus
}
#endif