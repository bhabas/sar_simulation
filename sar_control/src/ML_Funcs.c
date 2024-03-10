#include "ML_Funcs.h"

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

    // INITIALIZE NETWORK WEIGHTS AND BIASES FROM HEADER FILE VALUES
    for (int i = 0; i <= NN->num_layers; i++)
    {
        array_str = strtok_r(NULL,"*",&save_ptr);
        NN->W[i] = nml_mat_fromstr(array_str); // Weights

        array_str = strtok_r(NULL,"*",&save_ptr);
        NN->b[i] = nml_mat_fromstr(array_str); // Biases
    }

}

void NN_forward(nml_mat* X_input, nml_mat* Y_output, NN* NN)
{
    // LAYER 1
    // a = Leaky_Relu(W*X+b)
    nml_mat *WX1 = nml_mat_dot(NN->W[0],X_input); 
    nml_mat_add_r(WX1,NN->b[0]);
    nml_mat *a1 = nml_mat_funcElement(WX1,Leaky_Relu);

    // LAYER 2
    // a = Leaky_Relu(W*X+b)
    nml_mat *WX2 = nml_mat_dot(NN->W[1],a1); 
    nml_mat_add_r(WX2,NN->b[1]);
    nml_mat *a2 = nml_mat_funcElement(WX2,Leaky_Relu);

    // LAYER 3
    // a = Leaky_Relu(W*X+b)
    nml_mat *WX3 = nml_mat_dot(NN->W[2],a2); 
    nml_mat_add_r(WX3,NN->b[2]);
    nml_mat *a3 = nml_mat_funcElement(WX3,Leaky_Relu);

    // LAYER 4
    // a = W*X+b
    nml_mat *WX4 = nml_mat_dot(NN->W[3],a3); 
    nml_mat_add_r(WX4,NN->b[3]);
    nml_mat *a4 = nml_mat_cp(WX4);


    // SAVE NN OUTPUT
    Y_output->data[0][0] = a4->data[0][0];
    Y_output->data[1][0] = a4->data[1][0];
    Y_output->data[2][0] = expf(a4->data[2][0]);    // CONVERT LOG_STD TO STANDARD STD
    Y_output->data[3][0] = expf(a4->data[3][0]);    // *


    // FREE MATRICES FROM HEAP
    nml_mat_free(WX1);
    nml_mat_free(WX2);
    nml_mat_free(WX3);
    nml_mat_free(WX4);

    nml_mat_free(a1);
    nml_mat_free(a2);
    nml_mat_free(a3);
    nml_mat_free(a4);

}

float uniform_sample()
{
    return (float)rand()/(float)RAND_MAX;
}

float GaussianSample(float mu, float std)
{
    // Calc standard Gaussian sample via Central Limit Theorem
    float val = 0.0f;
    for (int i = 0; i < 12; i++)
    {
        val += uniform_sample();
    }
    val = val - 6;

    return val*std+mu;
    
}

float Sigmoid(float x)
{
    return 1/(1+expf(-x));
}

float Elu(float x)
{
    if(x>0) return x;

    else return expf(x)-1.0f;
 
}

float Pow2(float x)
{
    return pow(x,2);
 
}

float Relu(float x)
{
    if(x > 0.0f)
    {
        return x;
    }
    else
    {
        return 0.0f;
    }
}

float Leaky_Relu(float x) {

    float alpha = 0.01f;

    if (x > 0.0f) {
        return x;
    } else {
        return x * alpha;
    }
}

float scaleValue(float x, float original_min, float original_max, float target_min, float target_max) {
    // Scale x to [0, 1] in original range
    float x_scaled = (x - original_min) / (original_max - original_min);
    
    // Scale [0, 1] to target range
    float x_target = x_scaled * (target_max - target_min) + target_min;
    return x_target;
}