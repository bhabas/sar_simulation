#include <example.h>
#include "NN_Params.h"
// Compile Statement
// gcc example.c nml.c nml_util.c -I /home/bhabas/catkin_ws/src/crazyflie_simulation/crazyflie_gazebo/include  -Wall -o example -lm -Wall && ./example 


void initNN_Layers(Scaler* scaler,nml_mat* W[], nml_mat* b[], char path[],int numLayers)
{
    char array_list[2048];
    char* array_token;
    char* save_ptr;

    strcpy(array_list,str);


    array_token = strtok_r(array_list,"*",&save_ptr);
    scaler->mean = nml_mat_fromstr(array_token);
    array_token = strtok_r(NULL,"*",&save_ptr);

    scaler->std = nml_mat_fromstr(array_token);
    array_token = strtok_r(NULL,"*",&save_ptr);


    nml_mat_print(scaler->mean);
    nml_mat_print(scaler->std);

    for (int i = 0; i < numLayers; i++)
    {
        W[i] = nml_mat_fromstr(array_token);
        nml_mat_print(W[i]);
        array_token = strtok_r(NULL,"*",&save_ptr);
        b[i] = nml_mat_fromstr(array_token);
        nml_mat_print(b[i]);
        array_token = strtok_r(NULL,"*",&save_ptr);
    }

}

int main()
{   

    static Scaler scaler;

    static nml_mat* W[3];
    static nml_mat* b[3];

    initNN_Layers(&scaler,W,b,str,3);

    nml_mat_print(W[2]);

    return 0;
}


