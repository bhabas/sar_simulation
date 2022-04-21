#include <stdlib.h>
#include <stdio.h>
#include "nml.h"

int main()
{
    // SOLVE Ax = b

    double A[9] = {
        1.0, 2.0, 3.0,
        4.0, 5.0, 6.0,
        7.0, 8.0, 1.0,
    };

    double b[3] = {5.0,3.0,1.0};
    
    nml_mat* m_A = nml_mat_from(3,3,9,A);
    nml_mat* m_b = nml_mat_from(3,1,3,b);
    nml_mat_lup *LUP = nml_mat_lup_solve(m_A);

    nml_mat *x = nml_ls_solve(LUP, m_b);
    nml_mat_print(x);



}