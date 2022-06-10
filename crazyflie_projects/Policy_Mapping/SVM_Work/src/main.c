#include <stdio.h>

#include "NN_funcs.h"
#include "nml.h"

#include "NN_Params/SVM_Params_NL_DR.h"
#include "NN_Params/NN_Layers_NL_DR.h"

// gcc -o output -Wall src/main.c src/print1.c src/nml.c src/nml_util.c -I include -lm -g && ./output 

SVM SVM_PolicyFlip;     

// ===============================
//  NEURAL NETWORK INITIALIZATION
// ===============================
static nml_mat* X;  // STATE MATRIX TO BE INPUT INTO NN
static nml_mat* y_output;  // STATE MATRIX TO BE INPUT INTO NN

// NN INPUT SCALERS
static Scaler Scaler_Flip;      // Scale input vector for NN

// NN WEIGHTS
static nml_mat* W_flip[4];  

// NN BIASES
static nml_mat* b_flip[4];  

// NN OUTPUTS
float NN_flip = 0.0f;           // NN output value for flip classification
float NN_policy = 0.0f;         // NN output value for policy My

// NN OUTPUTS AT FLIP TRIGGER
float NN_tr_policy = 0.0f;      // NN policy value at flip trigger

int main()
{
    printf("Hello World!\n");
    X = nml_mat_new(3,1);
    X->data[0][0] = 0.29;
    X->data[1][0] = -0.673;
    X->data[2][0] = 0.952; 

    y_output = nml_mat_new(2,1);

    initNN_Layers(&Scaler_Flip,W_flip,b_flip,NN_Params_Flip,4);
    printf("NN_Predict: %.4f\n",NN_Forward_Flip(X,&Scaler_Flip,W_flip,b_flip));


    OC_SVM_init(&SVM_PolicyFlip,SVM_Params);

    printf("OC_SVM Predict: %.4f\n",OC_SVM_predict(&SVM_PolicyFlip,X));

    return 0;
}