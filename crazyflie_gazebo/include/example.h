#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "nml.h"


typedef struct{
    nml_mat* mean;
    nml_mat* std;
}Scaler;
