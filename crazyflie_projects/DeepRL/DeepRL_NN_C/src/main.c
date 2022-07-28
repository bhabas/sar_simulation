#include <stdio.h>

#include "nml.h"
#include "ML_funcs.h"
#include "NN_Params/NN_Layers_NL_DeepRL.h"

// gcc -o output -Wall src/main.c src/print1.c src/nml.c src/nml_util.c -I include -lm -g && ./output 


NN NN_DeepRL;


// ===============================
//  NEURAL NETWORK INITIALIZATION
// ===============================
nml_mat* X;  // STATE MATRIX TO BE INPUT INTO NN
nml_mat* y_output;

void __init__()
{
    X = nml_mat_new(3,1);
    X->data[0][0] = 0.1655;
    X->data[1][0] = -2.3880;
    X->data[2][0] = 0.3551; 

    y_output = nml_mat_new(4,1);
    srand(time(NULL));
}

int main()
{
    __init__();
    
    NN_init(&NN_DeepRL,NN_Params_DeepRL);
    
    // nml_mat_print(y_output);
    // printf("NN_Predict: %.4f\n",NN_predict(X,&NN_Policy));

    while(1)
    {
        NN_predict_DeepRL(X,y_output,&NN_DeepRL);
        // printf("Val: %.5f\n",GaussianSample(3,0.1));
    }

    return 0;
}