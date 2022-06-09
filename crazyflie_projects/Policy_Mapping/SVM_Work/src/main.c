#include <stdio.h>

#include "NN_funcs.h"
#include "nml.h"

#include "NN_Params/SVM_Params_NL_DR.h"
// gcc -o output -Wall src/main.c src/print1.c src/nml.c src/nml_util.c -I include -lm -g && ./output 

SVM_object SVM;     

int main()
{
    printf("Hello World!\n");

    init_OC_SVM(&SVM,SVM_Params);
    nml_mat_print(SVM.support_vecs);
    printf("gamma: %.3f\n",SVM.gamma);

    return 0;
}