#include <math.h>
#include <stdio.h>

void printMatrix(int r,int c,float arr[r][c]);

int main()
{
    printf("Hello World\n");

    float a[2][2] = {{1.0f,2.0f},{3.0f,4.0f}};

    

    printMatrix(2,2,a);
    
    float b[4][4];
    return 0;
}

void printMatrix(int r,int c,float arr[r][c])
{
    for (int i = 0; i < r; i++)
    {
        for (int j = 0; j < c; j++)
        {
            printf("%.2f, ",arr[i][j]);
        }
        printf("\n");
        
    }
}