#include <stdio.h>
#include <stdlib.h>


int Sum(int array[], int length);

void Output(int array[], int length, int (*Function)(int array[], int length));

int main()
{
    int array[] = {2,4,6,8,10};
    Output(array,5,Sum);

    return 0;
}

int Sum(int array[], int length)
{
    int sum = 0;

    for (int i = 0; i < length; i++)
    {
        sum = sum + array[i];
    }
    return sum;
}

void Output(int array[], int length, int (*Function)(int array[], int length))
{
    printf("Result is: %d\n",Function(array,length));
}
