#include <stdio.h>
#include <stdbool.h>
#include "controller_gtc.h"

int main()
{
    printf("%d\n",val);

    while(true)
    {

        controller_gtc();
        printf("%d\n",val);

    };
    

    

    return 0;
}