#include <stdio.h>
#include "example.h"
#include "print1.h"
#include "nml.h"
// gcc -o output -Wall src/main.c src/print1.c src/nml.c src/nml_util.c -I include -lm -g && ./output 
int main()
{
    printf("Hello World!\n");
    printf("Val: %.3f\n",val);
    print1();

    nml_mat* mat = nml_mat_new(3,3);
    nml_mat_print(mat);

    return 0;
}