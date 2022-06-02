// STANDARD LIBRARIES
#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "nml.h"
int main()
{
    printf("hello\n");
    nml_mat* mat = nml_mat_rnd(4,4,0,1);
    nml_mat_print(mat);

    return 1;
}