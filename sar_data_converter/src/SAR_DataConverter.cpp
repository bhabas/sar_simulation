#include "SAR_DataConverter.h"

void SAR_DC::method_a(int foo1)
{ 
    printf("Testing a\n");

}

int main(int argc, char** argv)
{
    SAR_DC SARDC;
    SARDC.method_a(4);
    SARDC.method_b(4);

    return 0;
}