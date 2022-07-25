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


int main()
{
   
    
    
    printf("size: %ld\n",sizeof(NN_Policy));
    X = nml_mat_new(3,1);
    X->data[0][0] = 0.0;
    X->data[1][0] = 1.0;
    X->data[2][0] = 2.0; 


    NN_init(&NN_Policy,NN_Params_Flip);
    OC_SVM_init(&SVM_PolicyFlip,SVM_Params);

    printf("NN_Predict: %.4f\n",NN_predict(X,&NN_Policy));
    printf("OC_SVM Predict: %.4f\n",OC_SVM_predict(X,&SVM_PolicyFlip));

    return 0;
}