#include <stdlib.h>
#include <stdio.h>
#include "nml.h"

int main()
{
    // SOLVE Ax = b

    double A[9] = {
        -1, 2, 4,
         5, 6, 6,
        -3, 5, 9
    };

    double b[3] = {1,2,3};
    
    nml_mat* m_A = nml_mat_from(3,3,9,A);
    nml_mat* m_b = nml_mat_from(3,1,3,b);

    // LUP SOLVE
    nml_mat_lup *LUP = nml_mat_lup_solve(m_A);
    nml_mat *x_LUP = nml_ls_solve(LUP, m_b);
    nml_mat_print(x_LUP);

    // QR SOLVE
    nml_mat_qr *QR = nml_mat_qr_solve(m_A); // A = Q*R
    nml_mat* y = nml_mat_dot(nml_mat_transp(QR->Q),m_b); // y = Q^T*b
    nml_mat* x_QR = nml_ls_solvebck(QR->R,y); // Solve R*x = y via back substitution
    nml_mat_print(x_QR);




}