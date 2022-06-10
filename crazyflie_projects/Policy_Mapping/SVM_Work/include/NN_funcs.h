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

typedef struct{
    nml_mat* scaler_mean;
    nml_mat* scaler_std;
    nml_mat* dual_coeffs;
    nml_mat* support_vecs;
    float gamma;
    float intercept;
}SVM_object;


// NEURAL NETWORK PRIMITIVES
void initNN_Layers(Scaler* scaler,nml_mat* W[], nml_mat* b[], char str[],int numLayers);
float Sigmoid(float x);
float Elu(float x);
float Pow2(float x);
void NN_Forward_Flip(nml_mat* X, nml_mat* y_output, Scaler* scaler, nml_mat* W[], nml_mat* b[]);

void init_OC_SVM(SVM_object* SVM,char str[]);
float SVM_predict(SVM_object* SVM,nml_mat* X);

float SVM_predict(SVM_object* SVM,nml_mat* X)
{
    // SCALE INPUT DATA
    nml_mat* X_input = nml_mat_cp(X);
    for(int i=0;i<3;i++)
    {
        X_input->data[i][0] = (X->data[i][0] - SVM->scaler_mean->data[i][0])/SVM->scaler_std->data[i][0];
    }


    // PASS INPUT THROUGH SVM
    nml_mat* tmp;
    double val = 0.0;
    double tmp_val = 0.0;

    for (int i = 0; i < SVM->support_vecs->num_cols; i++)
    {
        tmp = nml_mat_sub(X_input,nml_mat_col_get(SVM->support_vecs, i));
        tmp_val = nml_mat_sum_elem(nml_mat_funcElement(tmp,Pow2));
        val += SVM->dual_coeffs->data[i][0]*exp(-SVM->gamma*tmp_val);
    }
    val += SVM->intercept;
    
    nml_mat_free(X_input);
    nml_mat_free(tmp);

    return val;
}

void init_OC_SVM(SVM_object* SVM,char str[])
{
    char* array_str;
    char* save_ptr;
    nml_mat* tmp;

    // PARSE HEADER FILE FOR SCALER VALUES
    array_str = strtok_r(str,"*",&save_ptr);
    SVM->scaler_mean = nml_mat_fromstr(array_str);

    array_str = strtok_r(NULL,"*",&save_ptr);
    SVM->scaler_std = nml_mat_fromstr(array_str);

    // GAMMA
    array_str = strtok_r(NULL,"*",&save_ptr); 
    tmp = nml_mat_fromstr(array_str);
    SVM->gamma = tmp->data[0][0];

    // INTERCEPT
    array_str = strtok_r(NULL,"*",&save_ptr); 
    tmp = nml_mat_fromstr(array_str);
    SVM->intercept = tmp->data[0][0];

    // DUAL COEFFS
    array_str = strtok_r(NULL,"*",&save_ptr); 
    SVM->dual_coeffs = nml_mat_fromstr(array_str);

    // SUPPORT VECTORS
    array_str = strtok_r(NULL,"*",&save_ptr); 
    SVM->support_vecs = nml_mat_transp(nml_mat_fromstr(array_str));

    nml_mat_free(tmp);
}

void initNN_Layers(Scaler* scaler,nml_mat* W[], nml_mat* b[], char str[],int numLayers)
{
    char* array_str;
    char* save_ptr;

    // PARSE HEADER FILE FOR SCALER VALUES
    array_str = strtok_r(str,"*",&save_ptr);
    scaler->mean = nml_mat_fromstr(array_str);

    array_str = strtok_r(NULL,"*",&save_ptr);
    scaler->std = nml_mat_fromstr(array_str);

    array_str = strtok_r(NULL,"*",&save_ptr); // Load next array string

    // INITIALIZE NETWORK WEIGHTS AND BIASES FROM HEADER FILE VALUES
    for (int i = 0; i < numLayers; i++)
    {
        W[i] = nml_mat_fromstr(array_str); // Weights
        array_str = strtok_r(NULL,"*",&save_ptr);

        b[i] = nml_mat_fromstr(array_str); // Biases
        array_str = strtok_r(NULL,"*",&save_ptr);
    }


}


void NN_Forward_Flip(nml_mat* X, nml_mat* y_output, Scaler* scaler, nml_mat* W[], nml_mat* b[])
{
    // SCALE INPUT DATA
    nml_mat* X_input = nml_mat_cp(X);
    for(int i=0;i<3;i++)
    {
        X_input->data[i][0] = (X->data[i][0] - scaler->mean->data[i][0])/scaler->std->data[i][0];
    }

    // PASS DATA THROUGH NETWORK
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
    nml_mat *a3 = nml_mat_funcElement(WX3,Elu);


    // LAYER 4
    // Sigmoid(W*X+b)
    nml_mat *WX4 = nml_mat_dot(W[3],a3); 
    nml_mat_add_r(WX4,b[3]);
    

    // nml_mat_print(a4);




    // SAVE OUTPUT VALUE
    y_output->data[0][0] = Sigmoid(WX4->data[0][0]);
    y_output->data[1][0] = WX4->data[1][0];



    // FREE MATRICES FROM STACK
    nml_mat_free(X_input);
    nml_mat_free(WX1);
    nml_mat_free(WX2);
    nml_mat_free(WX3);
    nml_mat_free(WX4);


    nml_mat_free(a1);
    nml_mat_free(a2);
    nml_mat_free(a3);

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

float Pow2(float x)
{
    return pow(x,2);
 
}