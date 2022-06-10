#include <stdio.h>

#include "NN_funcs.h"
#include "nml.h"

#include "NN_Params/SVM_Params_NL_DR.h"
// gcc -o output -Wall src/main.c src/print1.c src/nml.c src/nml_util.c -I include -lm -g && ./output 

SVM SVM_PolicyFlip;     
nml_mat* X;

int main()
{
    printf("Hello World!\n");
    X = nml_mat_new(3,1);
    X->data[0][0] = 0.29;
    X->data[1][0] = -0.673;
    X->data[2][0] = 0.952; 


    OC_SVM_init(&SVM_PolicyFlip,SVM_Params);

    printf("OC_SVM Predict: %.4f\n",OC_SVM_predict(&SVM_PolicyFlip,X));

    return 0;
}