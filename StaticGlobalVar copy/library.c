#include <stdio.h>
#include "library.h"

static int number = 0;

void add_one()
{
    number++;
}

void print()
{
    printf("number: %d\n",number);
}