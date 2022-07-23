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
float NN_predict(nml_mat* X_input, NN* NN);

// OC_SVM PRIMITIVES
void OC_SVM_init(SVM* SVM, char str[]); 
float OC_SVM_predict(nml_mat* X_input, SVM* SVM);
nml_mat* RBF_Kernel(nml_mat* X, SVM* SVM);

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
    SVM->support_vecs = nml_mat_fromstr(array_str);

    nml_mat_free(tmp);
}

float OC_SVM_predict(nml_mat* X_input, SVM* SVM)
{
    // SCALE INPUT DATA
    nml_mat* X = nml_mat_new(X_input->num_cols,X_input->num_rows);
    for(int i=0;i<3;i++)
    {
        // Scale data to zero-mean and unit variance
        X->data[0][i] = (X_input->data[i][0] - SVM->scaler_mean->data[i][0]) / SVM->scaler_std->data[i][0];
    }

    // https://scikit-learn.org/stable/modules/svm.html
    // Eq: Decision_Value = sum(dual_coeff[ii]*K(X,supp_vec[ii])) + b
    // RBF Kernel: K(x,x') = exp(-gamma*||x-x'||**2)
    nml_mat* kernel_vec = RBF_Kernel(X,SVM);
    double y_pred = nml_vect_dot(SVM->dual_coeffs,0,kernel_vec,0) + SVM->intercept;
        
    // FREE MATRIX POINTERS
    nml_mat_free(X);
    nml_mat_free(kernel_vec);
    
    return y_pred;
}

nml_mat* RBF_Kernel(nml_mat* X, SVM* SVM)
{
    // CALC: (X-X')
    nml_mat* X_tmp = extend_row_vec(X,SVM->support_vecs->num_rows);
    nml_mat_sub_r(X_tmp,SVM->support_vecs);

    // CALC: -gamma*||X-X'||**2 
    // NOTE: ||X-X'||**2 = sum of squares (d_1^2 + d_2^2 + d_3^2 + ...)
    nml_mat* tmp = nml_mat_new(X_tmp->num_rows,1);
    double square_sum;
    for (int i = 0; i < X_tmp->num_rows; i++)
    {
        square_sum = 0.0;
        for (int j = 0; j < X_tmp->num_cols; j++)
        {
            square_sum += X_tmp->data[i][j]*X_tmp->data[i][j];
        }
        tmp->data[i][0] = -SVM->gamma*square_sum;
    }

    // CALC: K(X,X') = exp(-gamma*||X-X'||**2)
    nml_mat* kernel_vec = nml_mat_funcElement(tmp,expf);
    
    // FREE ALLOCATED MEMORY
    nml_mat_free(X_tmp);
    nml_mat_free(tmp);

    return kernel_vec;
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

    //LAYER 1
    //Elu(W*X+b)
    nml_mat *WX1 = nml_mat_dot(NN->W[0],X); 
    nml_mat_add_r(WX1,NN->b[0]);
    nml_mat *a1 = nml_mat_funcElement(WX1,Elu);

    // LAYER 2
    // Elu(W*X+b)
    nml_mat *WX2 = nml_mat_dot(NN->W[1],a1); 
    nml_mat_add_r(WX2,NN->b[1]);
    nml_mat *a2 = nml_mat_funcElement(WX2,Elu);

    // LAYER 3
    // Elu(W*X+b)
    nml_mat *WX3 = nml_mat_dot(NN->W[2],a2); 
    nml_mat_add_r(WX3,NN->b[2]);
    nml_mat *a3 = nml_mat_funcElement(WX3,Elu);

    // LAYER 4
    // W*X+b
    nml_mat *WX4 = nml_mat_dot(NN->W[3],a3); 
    nml_mat_add_r(WX4,NN->b[3]);
    nml_mat *a4 = nml_mat_cp(WX4);


    // SAVE OUTPUT VALUE
    float y_output = (float)a4->data[0][0];

    // FREE MATRICES FROM STACK
    nml_mat_free(X);
    nml_mat_free(WX1);
    nml_mat_free(WX2);
    nml_mat_free(WX3);
    nml_mat_free(WX4);

    nml_mat_free(a1);
    nml_mat_free(a2);
    nml_mat_free(a3);
    nml_mat_free(a4);

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

