// STANDARD LIBRARIES
#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "nml.h"
#include "NN_funcs.h"

#include "NN_Params/SVM_Params_NL_DR.h"

static Scaler Scaler_Flip;      // Scale input vector for NN

float gamma_ = 0.5f;
float intercept = 0.0f;
static nml_mat* dual_coeffs;  
static nml_mat* supp_vecs;

int main()
{
    printf("hello\n");
    nml_mat* mat = nml_mat_rnd(4,4,0,1);
    nml_mat_print(mat);

    init_OC_SVM(&Scaler_Flip,SVM_Params,dual_coeffs,supp_vecs,intercept,gamma_);

    return 1;
}