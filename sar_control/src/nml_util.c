#include "nml_util.h"

float nml_rand_interval(float min, float max) {
  float d;
  d = (float) rand() / ((float) RAND_MAX + 1);
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