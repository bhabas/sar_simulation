#include "CF.h"

void CFDC::method_a(int foo1)
{ 
    printf("Testing a\n");

}

int main(int argc, char** argv)
{
    CFDC CF_DC;
    CF_DC.method_a(4);
    CF_DC.method_b(4);

    return 0;
}