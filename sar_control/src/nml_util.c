#include "nml_util.h"

double nml_rand_interval(double min, double max) {
  double d;
  d = (double) rand() / ((double) RAND_MAX + 1);
  return (min + d * (max - min));
}

void nml_log(const char* str)
{
    DEBUG_PRINT("%s %s %s",YELLOW,str,RESET);
}

void NP_CHECK(void* ptr) {
    if(ptr == NULL) {
        DEBUG_PRINT("%s Pointer is NULL. %s\n",YELLOW,RESET);
    } 
}