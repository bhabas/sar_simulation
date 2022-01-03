#include <math.h>
#include <stdio.h>
#include <stdlib.h>


void printMatrix(float **arr, int m, int n);



int main()
{

    // float a[2][2] = {{1.0f,2.0f},{3.0f,4.0f}};
    // printMatrix(2,2,a);

    // DEFINE ARRAY SIZE
    int m = 1;
    int n = 3;

    // DYNAMICALLY ALLOCATE MEMORY IN HEAP FOR ARRAY
    float **W = (float **)malloc(m*sizeof(float *));
    for (int i = 0; i < m; i++)
    {
        W[i] = (float *)malloc(n*sizeof(float));
    }
    
    float **X = (float **)malloc(3*sizeof(float *));
    for (int i = 0; i < 3; i++)
    {
        X[i] = (float *)malloc(1*sizeof(float));
    }

    // ASSIGN ARRAY ELEMENTS
    // for(int i=0;i<m;i++)
    // {
    //     for(int j=0;j<n;j++)
    //     {
    //         A[i][j] = i + j;
    //     }
    // }
    W[0][0] = -0.1548f;
    W[0][1] =  0.2339f;
    W[0][2] = -0.2190f;

    X[0][0] = 1.0f;
    X[1][0] = 2.0f;
    X[2][0] = 3.0f;

    printMatrix(W,m,n);
    printf("\n");
    printMatrix(X,3,1);

    // deallocate memory
    free(*W);
    free(W);

    free(*X);
    free(X);


    float b[4][4];
    return 0;
}

void printMatrix(float **arr, int m, int n)
{
    for (int i = 0; i < m; i++)
    {
        for (int j = 0; j < n; j++)
        {
            printf("%.2f ",arr[i][j]);
        }
        printf("\n");
        
    }
}