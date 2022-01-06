#include <stdio.h>
#include <stdlib.h>
#include <float.h>
#include <math.h>
#include <stdarg.h>
#include <string.h>


typedef struct nml_mat_s{
    int num_rows;
    int num_cols;
    int **data;
    int is_square;
} nml_mat;

nml_mat* nml_mat_new(int num_rows, int num_cols);

void printMat(nml_mat* m);
void addMat(nml_mat* m1, nml_mat* m2);


int main(){

    

    nml_mat* X = nml_mat_new(3,1);

    X->data[0][0] = 0;
    X->data[1][0] = 1;
    X->data[2][0] = 2;


   

    nml_mat* b = nml_mat_new(4,1);

    b->data[0][0] = 4;
    b->data[1][0] = 5;
    b->data[2][0] = 6;
    b->data[3][0] = 7;


   

    nml_mat* W = nml_mat_new(4,3);


    W->data[0][0] = 1;
    W->data[0][1] = 1;
    W->data[0][2] = 1;

    W->data[1][0] = 1;
    W->data[1][1] = 1;
    W->data[1][2] = 1;

    W->data[2][0] = 1;
    W->data[2][1] = 1;
    W->data[2][2] = 1;

    W->data[3][0] = 1;
    W->data[3][1] = 1;
    W->data[3][2] = 1;

    

    nml_mat* val = nml_mat_new(4,1);


    val->data[0][0] = 8;
    val->data[1][0] = 9;
    val->data[2][0] = 10;
    val->data[3][0] = 11;

    

    

    printMat(X);
    printMat(b);
    printMat(W);
    printMat(val);
    
    // // PRINT MATRIX3
    // for (int i = 0; i < 4; i++)
    // {
    //     for (int j = 0; j < 4; j++)
    //     {
    //         printf("%d\t\t",W->data[i][j]);
    //     }
    //     printf("\n");   
    // }
    // printf("\n");

    for(int i = 0; i < val->num_rows; i++) {
        for(int j = 0; j < val->num_cols; j++) {
            for(int k = 0; k < W->num_cols; k++) {
                val->data[i][j] += W->data[i][k] * X->data[k][j];
            }
        }
    }

   

    printMat(val);
    addMat(val,b);
    printMat(val);






    // addMat(X,b);
    
    
    // printMat(X);



    for (int i = 0; i < 3; i++)
    {
        free(X->data[i]);
    }
    free(X->data);
    free(X);
    
    for (int i = 0; i < 4; i++)
    {
        free(b->data[i]);
    }
    free(b->data);
    free(b);
    
    for (int i = 0; i < 4; i++)
    {
        free(W->data[i]);
    }
    free(W->data);
    free(W);

    for (int i = 0; i < 4; i++)
    {
        free(val->data[i]);
    }
    free(val->data);
    free(val);
    
   

   

    

    


    return 0;
}

void addMat(nml_mat* m1, nml_mat* m2)
{
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 1; j++)
        {
            m1->data[i][j] += m2->data[i][j];
        }
        
    }
}

void printMat(nml_mat* m)
{
    // PRINT MATRIX
    for (int i = 0; i < m->num_rows; i++)
    {
        for (int j = 0; j < m->num_cols; j++)
        {
            printf("%d\t",m->data[i][j]);
        }
        printf("\n");   
    }
    printf("\n");


}


nml_mat* nml_mat_new(int num_rows, int num_cols)
{
    nml_mat* X = malloc(sizeof(nml_mat));
    X->num_rows = num_rows;
    X->num_cols = num_cols;
    X->is_square = 0;
    X->data = malloc(num_rows*sizeof(int*));
    for(int i=0; i<num_rows;i++)
    {
        X->data[i] = malloc(num_cols*sizeof(int));
    }

    return X;
    

}