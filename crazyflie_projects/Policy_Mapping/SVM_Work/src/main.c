#include <stdio.h>

#include "NN_funcs.h"
#include "nml.h"

#include "NN_Params/SVM_Params_NL_DR.h"
// gcc -o output -Wall src/main.c src/print1.c src/nml.c src/nml_util.c -I include -lm -g && ./output 

SVM_object SVM;     
nml_mat* X;

int main()
{
    printf("Hello World!\n");
    X = nml_mat_new(3,1);
    X->data[0][0] = 5;
    X->data[1][0] = 2;
    X->data[2][0] = 1; 

    init_OC_SVM(&SVM,SVM_Params);
    float valx = SVM_predict(&SVM,X);

    printf("OC_SVM Predict: %.3f\n",valx);

    return 0;
}