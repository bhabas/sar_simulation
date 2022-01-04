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
    const char *f_W = "data/W.data";
    nml_mat *W = nml_mat_fromfile(f_W);

    const char *f_X = "data/X.data";
    nml_mat *X = nml_mat_fromfile(f_X);

    const char *f_b = "data/b.data";
    nml_mat *b = nml_mat_fromfile(f_b);


    // W*X + b
    nml_mat *m3 = nml_mat_dot(W, X);
    m3 = nml_mat_add(m3,b);
    nml_mat_print(m3);


    nml_mat *q = nml_mat_funcElement(m3,Sigmoid);
    nml_mat_print(q);


    nml_mat_free(W);
    nml_mat_free(X);
    nml_mat_free(b);
    // nml_mat_free(m3);



    return 0;
}

