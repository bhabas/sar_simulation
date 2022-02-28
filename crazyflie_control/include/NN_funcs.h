// STANDARD LIBRARIES
#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "nml.h"


typedef struct{
    nml_mat* mean;
    nml_mat* std;
}Scaler;

// NEURAL NETWORK PRIMITIVES
void initNN_Layers(Scaler* scaler,nml_mat* W[], nml_mat* b[], char str[],int numLayers);
float Sigmoid(float x);
float Elu(float x);
float NN_Forward_Policy(nml_mat* X, Scaler* scaler, nml_mat* W[], nml_mat* b[]);
float NN_Forward_Flip(nml_mat* X, Scaler* scaler, nml_mat* W[], nml_mat* b[]);



void initNN_Layers(Scaler* scaler,nml_mat* W[], nml_mat* b[], char str[],int numLayers)
{
    char* array_token;
    char* save_ptr;

    array_token = strtok_r(str,"*",&save_ptr);
    scaler->mean = nml_mat_fromstr(array_token);
    array_token = strtok_r(NULL,"*",&save_ptr);

    scaler->std = nml_mat_fromstr(array_token);
    array_token = strtok_r(NULL,"*",&save_ptr);


    for (int i = 0; i < numLayers; i++)
    {
        W[i] = nml_mat_fromstr(array_token);
        array_token = strtok_r(NULL,"*",&save_ptr);
        b[i] = nml_mat_fromstr(array_token);
        array_token = strtok_r(NULL,"*",&save_ptr);
    }


}

float NN_Forward_Policy(nml_mat* X, Scaler* scaler, nml_mat* W[], nml_mat* b[])
{   
    nml_mat* X_input = nml_mat_cp(X);

    // X_input = nml_mat_divEl(nml_mat_subEl(X-scaler->mean),scaler->std);
    for(int i=0;i<3;i++)
    {
        X_input->data[i][0] = (X->data[i][0] - scaler->mean->data[i][0])/scaler->std->data[i][0];
    }

    //LAYER 1
    //Sigmoid(W*X+b)
    nml_mat *WX1 = nml_mat_dot(W[0],X_input); 
    nml_mat_add_r(WX1,b[0]);
    nml_mat *a1 = nml_mat_funcElement(WX1,Sigmoid);

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


    // SAVE OUTPUT VALUE
    float y_output = (float)a3->data[0][0];

    // FREE MATRICES FROM STACK
    nml_mat_free(X_input);
    nml_mat_free(WX1);
    nml_mat_free(WX2);
    nml_mat_free(WX3);

    nml_mat_free(a1);
    nml_mat_free(a2);
    nml_mat_free(a3);


    return y_output;
}

float NN_Forward_Flip(nml_mat* X, Scaler* scaler, nml_mat* W[], nml_mat* b[])
{
    nml_mat* X_input = nml_mat_cp(X);
    for(int i=0;i<3;i++)
    {
        X_input->data[i][0] = (X->data[i][0] - scaler->mean->data[i][0])/scaler->std->data[i][0];
    }


    // LAYER 1
    // Elu(W*X+b)
    nml_mat *WX1 = nml_mat_dot(W[0],X_input); 
    nml_mat_add_r(WX1,b[0]);
    nml_mat *a1 = nml_mat_funcElement(WX1,Elu);


    // LAYER 2
    // Elu(W*X+b)
    nml_mat *WX2 = nml_mat_dot(W[1],a1); 
    nml_mat_add_r(WX2,b[1]);
    nml_mat *a2 = nml_mat_funcElement(WX2,Elu);


    // LAYER 3
    // Sigmoid(W*X+b)
    nml_mat *WX3 = nml_mat_dot(W[2],a2); 
    nml_mat_add_r(WX3,b[2]);
    nml_mat *a3 = nml_mat_funcElement(WX3,Sigmoid);




    // // SAVE OUTPUT VALUE
    float y_output = a3->data[0][0];


    // // FREE MATRICES FROM STACK
    nml_mat_free(X_input);
    nml_mat_free(WX1);
    nml_mat_free(WX2);
    nml_mat_free(WX3);

    nml_mat_free(a1);
    nml_mat_free(a2);
    nml_mat_free(a3);

    return y_output;
}

float Sigmoid(float x)
{
    return 1/(1+exp(-x));
}

float Elu(float x)
{
    if(x>0) return x;

    else return exp(x)-1.0f;
 
}