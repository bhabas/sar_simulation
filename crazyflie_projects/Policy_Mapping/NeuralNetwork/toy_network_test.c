#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <math.h>

#include "nml.h"
float Sigmoid(float x)
{
    return 1/(1+exp(-x));
    // return 2*x;;
}


int main()
{
    double X1_array[3] = {-1.6974,  0.4014, -1.1264}; // 4x1
    nml_mat *X1 = nml_mat_from(3, 1, 3, X1_array);

    FILE *input = fopen("data/NN_Layers.data", "r");

   
    // LAYER 1
    nml_mat *W1 = nml_mat_fromfilef(input);
    nml_mat *b1 = nml_mat_fromfilef(input);


    // LAYER 2
    nml_mat *W2 = nml_mat_fromfilef(input);
    nml_mat *b2 = nml_mat_fromfilef(input);

    // LAYER 3
    nml_mat *W3 = nml_mat_fromfilef(input);
    nml_mat *b3 = nml_mat_fromfilef(input);



    fclose(input);


    // LAYER 1
    nml_mat *WX = nml_mat_dot(W1,X1); // 4x3 * 3x1 = 4x1
    nml_mat_add_r(WX,b1);
    nml_mat *a1 = nml_mat_funcElement(WX,Sigmoid);

    // LAYER 2
    nml_mat *WX2 = nml_mat_dot(W2,a1); 
    nml_mat_add_r(WX2,b2);
    nml_mat *a2 = nml_mat_funcElement(WX2,Sigmoid);

    // LAYER 3
    nml_mat *WX3 = nml_mat_dot(W3,a2); 
    nml_mat_add_r(WX3,b3);
    nml_mat *y = WX3;


    // nml_mat_print(WX);
    // nml_mat_print(b1);
    // nml_mat_print(a1);
    
    nml_mat_print(y);


    // nml_mat_free(W1);
    // nml_mat_free(X1);
    // nml_mat_free(b1);
    // nml_mat_free(WX);
    // nml_mat_free(a1);

    // nml_mat_free(a2);
    // nml_mat_free(WX2);


    return 0;
}

