#include <stdio.h>

#include "NN_funcs2.h"
#include "nml.h"

#include "NN_Params/SVM_Params_NL_DR.h"
#include "NN_Params/NN_Layers_NL_DR.h"

// gcc -o output -Wall src/main.c src/print1.c src/nml.c src/nml_util.c -I include -lm -g && ./output 


SVM SVM_PolicyFlip;     
NN NN_Policy;


// ===============================
//  NEURAL NETWORK INITIALIZATION
// ===============================
static nml_mat* X;  // STATE MATRIX TO BE INPUT INTO NN
static nml_mat* y_output;  // STATE MATRIX TO BE INPUT INTO NN


int main()
{
   
    

    printf("Hello World!\n");
    X = nml_mat_new(3,1);
    X->data[0][0] = 0.29;
    X->data[1][0] = -0.673;
    X->data[2][0] = 0.952; 

    y_output = nml_mat_new(2,1);

    NN_init(&NN_Policy,NN_Params_Flip);
    NN_predict(X,&NN_Policy);
    printf("NN_Predict: %.4f\n",NN_predict(X,&NN_Policy));


    OC_SVM_init(&SVM_PolicyFlip,SVM_Params);

    printf("OC_SVM Predict: %.4f\n",OC_SVM_predict(&SVM_PolicyFlip,X));

    return 0;
}