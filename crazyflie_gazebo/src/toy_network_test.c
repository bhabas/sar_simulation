#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <math.h>

#include "nml.h"
float Sigmoid(float x)
{
    return 1/(1+exp(-x));
}

double NN_Output(nml_mat* X, nml_mat* W[], nml_mat* b[]);

int main()
{
    double X1_array[3] = {-1.6974,  0.4014, -1.1264}; // 4x1
    nml_mat *X = nml_mat_from(3, 1, 3, X1_array);

    FILE *input = fopen("/home/bhabas/catkin_ws/src/crazyflie_simulation/crazyflie_gazebo/src/data/NN_Layers.data", "r");

   
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


    nml_mat* W[3] = {W1,W2,W3};
    nml_mat* b[3] = {b1,b2,b3};


    double y = NN_Output(X,W,b);




    printf("y_output: %.4f\n",y);

    return 0;
}


double NN_Output(nml_mat* X, nml_mat* W[], nml_mat* b[])
{

    // LAYER 1
    // Sigmoid(W*X+b)
    nml_mat *WX = nml_mat_dot(W[0],X); 
    nml_mat_add_r(WX,b[0]);
    nml_mat *a1 = nml_mat_funcElement(WX,Sigmoid);

    // LAYER 2
    // Sigmoid(W*X+b)
    nml_mat *WX2 = nml_mat_dot(W[1],a1); 
    nml_mat_add_r(WX2,b[1]);
    nml_mat *a2 = nml_mat_funcElement(WX2,Sigmoid);

    // LAYER 3
    // (W*X+b)
    nml_mat *WX3 = nml_mat_dot(W[2],a2); 
    nml_mat_add_r(WX3,b[2]);
    nml_mat *a3 = nml_mat_cp(WX3);

    double y_output = a3->data[0][0];

    nml_mat_free(WX);
    nml_mat_free(a1);
    nml_mat_free(WX2);
    nml_mat_free(a2);
    nml_mat_free(WX3);
    nml_mat_free(a3);


    return y_output;
}
