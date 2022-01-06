#include <stdio.h>
#include <stdlib.h>
#include <float.h>
#include <math.h>
#include <stdarg.h>
#include <string.h>


typedef struct nml_mat_s{
    int num_rows;
    int num_cols;
    double **data;
    int is_square;
} nml_mat;

nml_mat* nml_mat_new(int num_rows, int num_cols);
void nml_mat_free(nml_mat* m);

void nml_mat_print(nml_mat* m);
void addMat(nml_mat* m1, nml_mat* m2);
nml_mat* nml_mat_dot(nml_mat *m1, nml_mat *m2);
void nml_mat_funcElement(nml_mat *m, float (*Function)(float x));

float Sigmoid(float x)
{
    // return 1/(1+exp(-x));
    return 2*x;
}


int main(){

    

    nml_mat* X = nml_mat_new(4,1);

    X->data[0][0] = 0;
    X->data[1][0] = 1;
    X->data[2][0] = 2;
    X->data[3][0] = 3;



   

    nml_mat* b = nml_mat_new(5,1);

    b->data[0][0] = 4;
    b->data[1][0] = 5;
    b->data[2][0] = 6;
    b->data[3][0] = 7;
    b->data[4][0] = 8;



   

    nml_mat* W = nml_mat_new(5,4);


    W->data[0][0] = 1;
    W->data[0][1] = 1;
    W->data[0][2] = 1;
    W->data[0][3] = 1;

    W->data[1][0] = 1;
    W->data[1][1] = 1;
    W->data[1][2] = 1;
    W->data[1][3] = 1;

    W->data[2][0] = 1;
    W->data[2][1] = 1;
    W->data[2][2] = 1;
    W->data[2][3] = 1;

    W->data[3][0] = 1;
    W->data[3][1] = 1;
    W->data[3][2] = 1;
    W->data[3][3] = 1;

    W->data[4][0] = 1;
    W->data[4][1] = 1;
    W->data[4][2] = 1;
    W->data[4][3] = 1;

    

    

    

    

    nml_mat_print(X);
    nml_mat_print(W);
    nml_mat_print(b);


    nml_mat* r = nml_mat_dot(W,X);

   

    nml_mat_print(r);
    nml_mat_print(b);

    addMat(r,b);
    nml_mat_print(r);

    nml_mat_funcElement(r,Sigmoid);
    nml_mat_print(r);







    nml_mat_free(X);
    nml_mat_free(b);
    nml_mat_free(W);
    nml_mat_free(r);

    

    
   

   

    

    


    return 0;
}

void nml_mat_free(nml_mat* m)
{
    for (int i = 0; i < m->num_rows; i++)
    {
        free(m->data[i]);
    }
    free(m->data);
    free(m);

}

void addMat(nml_mat* m1, nml_mat* m2)
{
    for (int i = 0; i < m1->num_rows; i++)
    {
        for (int j = 0; j < m1->num_cols; j++)
        {
            m1->data[i][j] += m2->data[i][j];
        }
        
    }
}

nml_mat* nml_mat_dot(nml_mat *m1, nml_mat *m2)
{
    nml_mat* r = nml_mat_new(m1->num_rows, m2->num_cols);



    for(int i = 0; i < r->num_rows; i++) {
        for(int j = 0; j < r->num_cols; j++) {
            for(int k = 0; k < m1->num_cols; k++) {
                r->data[i][j] += m1->data[i][k] * m2->data[k][j];
            }
        }
    }

    return r;
}

void nml_mat_funcElement(nml_mat *m,float (*Function)(float x)) {

  int i, j;
  for(i = 0; i < m->num_rows; i++) {
    for(j = 0; j < m->num_cols; j++) {
      m->data[i][j] = Function(m->data[i][j]);
    }
  }

}


void nml_mat_print(nml_mat* m)
{
    // PRINT MATRIX
    for (int i = 0; i < m->num_rows; i++)
    {
        for (int j = 0; j < m->num_cols; j++)
        {
            printf("%.3f\t",m->data[i][j]);
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
    X->data = malloc(num_rows*sizeof(double*));
    for(int i=0; i<num_rows;i++)
    {
        X->data[i] = malloc(num_cols*sizeof(double));
    }

    return X;
    

}