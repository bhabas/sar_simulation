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



// NEURAL NETWORK FUNCTIONS
void NN_init(NN* NN_Policy, char str[]);
void NN_forward(nml_mat* X_input, nml_mat* y_output, NN* NN);
float scale_tanhAction(float action, float low, float high);

// SAMPLING FUNCTIONS
float uniform_sample();
float GaussianSample(float mu, float std);

// CUSTOM ELEMENT FUNCTIONS
float Sigmoid(float x);
float Elu(float x);
float Pow2(float x);
float Relu(float x);
float Leaky_Relu(float x);
