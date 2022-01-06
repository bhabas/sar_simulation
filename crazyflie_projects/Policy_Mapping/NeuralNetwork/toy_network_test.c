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
    double X1_array[3] = {1.0, 2.0, 3.0}; // 4x1
    nml_mat *X1 = nml_mat_from(3, 1, 3, X1_array);
   
    // LAYER 1
    const char *f_W1 = "data/W_1.data";
    nml_mat *W1 = nml_mat_fromfile(f_W1);

    const char *f_b1 = "data/b_1.data";
    nml_mat *b1 = nml_mat_fromfile(f_b1);

    // LAYER 2
    const char *f_W2 = "data/W_2.data";
    nml_mat *W2 = nml_mat_fromfile(f_W2);

    const char *f_b2 = "data/b_2.data";
    nml_mat *b2 = nml_mat_fromfile(f_b2);




    nml_mat *WX = nml_mat_dot(W1,X1); // 4x3 * 3x1 = 4x1
    nml_mat_add_r(WX,b1);
    nml_mat *a1 = nml_mat_funcElement(WX,Sigmoid);

    nml_mat_print(WX);
    nml_mat_print(b1);
    nml_mat_add_r(WX,b1);
    nml_mat_print(a1);


    nml_mat *WX2 = nml_mat_dot(W2,a1); 
    nml_mat_add_r(WX2,b2);
    nml_mat *a2 = nml_mat_funcElement(WX2,Sigmoid);


    
    nml_mat_print(a2);


    // nml_mat_free(W1);
    // nml_mat_free(X1);
    // nml_mat_free(b1);
    // nml_mat_free(WX);
    // nml_mat_free(a1);

    // nml_mat_free(a2);
    // nml_mat_free(WX2);


    return 0;
}

