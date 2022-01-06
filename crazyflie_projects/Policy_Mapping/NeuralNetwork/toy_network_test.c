#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <math.h>

#include "nml.h"
float Sigmoid(float x)
{
    // return 1/(1+exp(-x));
    return 2*x;;
}


int main()
{

    double W1_array[16] = {
        1.0, 2.0, 3.0, 4.0, 
        1.0, 2.0, 3.0, 4.0, 
        1.0, 2.0, 3.0, 4.0,
        1.0, 2.0, 3.0, 4.0}; // 4x4
    nml_mat *W1 = nml_mat_from(4, 4, 16, W1_array);

    double X1_array[4] = {1.0, 2.0, 3.0, 4.0}; // 4x1
    nml_mat *X1 = nml_mat_from(4, 1, 4, X1_array);

    nml_mat *WX = nml_mat_dot(W1,X1); // 4x3 * 3x1 = 4x1

    double b1_array[4] = {1.0, 2.0, 3.0, 4.0}; // 4x1
    nml_mat *b1 = nml_mat_from(4, 1, 4, b1_array);

    // nml_mat_add_r(WX,b1);

    // nml_mat *a1 = nml_mat_funcElement(WX,Sigmoid);


    nml_mat_print(WX);
    nml_mat_print(b1);
    nml_mat_add_r(WX,b1);

    nml_mat_free(W1);
    nml_mat_free(X1);
    nml_mat_free(b1);
    // nml_mat_free(WX);
    // nml_mat_free(a1);


    return 0;
}

