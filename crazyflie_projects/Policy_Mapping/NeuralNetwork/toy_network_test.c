#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <math.h>

#include "nml.h"
float Sigmoid(float x)
{
    return 1/(1+exp(-x));
}


int main()
{
    // DEFINE MATRICES FROM FILE
    const char *f_W1 = "data/W_1.data";
    nml_mat *W1 = nml_mat_fromfile(f_W1);

    const char *f_b1 = "data/b_1.data";
    nml_mat *b1 = nml_mat_fromfile(f_b1);


    const char *f_W2 = "data/W_2.data";
    nml_mat *W2 = nml_mat_fromfile(f_W2);

    const char *f_b2 = "data/b_2.data";
    nml_mat *b2 = nml_mat_fromfile(f_b2);



    double X_array[3] = {1.0, 2.0, 3.0};
    nml_mat *X = nml_mat_from(3, 1, 3, X_array);


    // a = sig(W*X + b)
    nml_mat *a1 = nml_mat_dot(W1, X);
    nml_mat_print(b1);
    // printf("%d",b1->num_rows);
    // nml_mat *a2 = nml_mat_add(a1,b1);
    // nml_mat *a3 = nml_mat_funcElement(a2,Sigmoid);

    // nml_mat_print(a1);

    nml_mat_free(X);

    nml_mat_free(W1);
    nml_mat_free(b1);
    nml_mat_free(W2);
    nml_mat_free(b2);
    nml_mat_free(a1);



    return 0;
}

