// STANDARD LIBRARIES
#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "nml.h"


typedef struct{
    uint8_t num_layers;
    nml_mat* scaler_mean;
    nml_mat* scaler_std;
    nml_mat* W[6];  // Weights
    nml_mat* b[6];  // biases
}NN;

typedef struct{
    nml_mat* scaler_mean;
    nml_mat* scaler_std;
    nml_mat* dual_coeffs;
    nml_mat* support_vecs;
    float gamma;
    float intercept;
}SVM;


// NEURAL NETWORK PRIMITIVES
void NN_init(NN* NN_Policy, char str[]);
float NN_predict(nml_mat* X, NN* NN);

// OC_SVM PRIMITIVES
void OC_SVM_init(SVM* SVM, char str[]); 
float OC_SVM_predict(SVM* SVM, nml_mat* X);

// CUSTOM ELEMENT FUNCTIONS
float Sigmoid(float x);
float Elu(float x);
float Pow2(float x);


void OC_SVM_init(SVM* SVM,char str[]) 
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

float OC_SVM_predict(SVM* SVM, nml_mat* X_input)
{
    // SCALE INPUT DATA
    nml_mat* X = nml_mat_cp(X_input);
    for(int i=0;i<3;i++)
    {
        // Scale data to zero-mean and unit variance
        X->data[i][0] = (X_input->data[i][0] - SVM->scaler_mean->data[i][0]) / SVM->scaler_std->data[i][0];
    }

    // PASS INPUT DATA THROUGH SVM
    nml_mat* tmp_mat;
    double tmp_val = 0.0;
    double SVM_pred = 0.0;

    // https://scikit-learn.org/stable/modules/svm.html
    // Eq: Decision_Value = sum(dual_coeff[ii]*K(supp_vec[ii],X)) + b
    // Kernel: K(x,x') = exp(-gamma*||x-x'||**2)
    for (int i = 0; i < SVM->support_vecs->num_cols; i++)
    {
        tmp_mat = nml_mat_sub(X,nml_mat_col_get(SVM->support_vecs, i));
        tmp_val = nml_mat_sum_elem(nml_mat_funcElement(tmp_mat,Pow2));
        SVM_pred += SVM->dual_coeffs->data[i][0]*exp(-SVM->gamma*tmp_val);
    }
    SVM_pred += SVM->intercept;
    
    // FREE MATRIX POINTERS
    nml_mat_free(X);
    nml_mat_free(tmp_mat);

    return SVM_pred;
}

void NN_init(NN* NN, char str[])
{
    char* array_str;
    char* save_ptr;

    // PARSE HEADER FILE FOR SCALER VALUES
    array_str = strtok_r(str,"*",&save_ptr);
    NN->num_layers = nml_mat_fromstr(array_str)->data[0][0];

    // PARSE HEADER FILE FOR SCALER VALUES
    array_str = strtok_r(NULL,"*",&save_ptr);
    NN->scaler_mean = nml_mat_fromstr(array_str);

    array_str = strtok_r(NULL,"*",&save_ptr);
    NN->scaler_std = nml_mat_fromstr(array_str);

    array_str = strtok_r(NULL,"*",&save_ptr); // Load next array string


    // INITIALIZE NETWORK WEIGHTS AND BIASES FROM HEADER FILE VALUES
    for (int i = 0; i < NN->num_layers; i++)
    {
        NN->W[i] = nml_mat_fromstr(array_str); // Weights
        array_str = strtok_r(NULL,"*",&save_ptr);

        NN->b[i] = nml_mat_fromstr(array_str); // Biases
        array_str = strtok_r(NULL,"*",&save_ptr);
    }

}

float NN_predict(nml_mat* X_input, NN* NN)
{
    // SCALE INPUT DATA
    nml_mat* X = nml_mat_cp(X_input);
    for(int i=0;i<3;i++)
    {
        // Scale data to zero-mean and unit variance
        X->data[i][0] = (X_input->data[i][0] - NN->scaler_mean->data[i][0]) / NN->scaler_std->data[i][0];
    }

    // PASS DATA THROUGH NETWORK
    nml_mat* WX;
    for (int i = 0; i < NN->num_layers; i++)
    { 
        WX = nml_mat_add(nml_mat_dot(NN->W[i],X),NN->b[i]); // WX+b

        if (i == NN->num_layers-1) // Last Layer
        {
            X = nml_mat_cp(WX); // X = WX+b
        }
        else 
        {
            X = nml_mat_funcElement(WX,Elu); // X = f(WX+b)
        }      

    }
    

    // SAVE OUTPUT VALUE
    float y_output = X->data[0][0];


    // FREE MATRICES FROM STACK
    nml_mat_free(X);
    nml_mat_free(WX);

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

float Pow2(float x)
{
    return pow(x,2);
 
}