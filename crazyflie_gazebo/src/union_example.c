#include <stdio.h>
#include <stdlib.h>
#include <float.h>
#include <math.h>
#include <stdarg.h>
#include <string.h>

struct vec2{
  union {
    struct {
      float x;
      float y;
      float z;
    };
    float vec[3];
  };
};

int main()
{   
    
    struct vec2 a;
    a.x = 1.0f;
    a.y = 2.0f;
    a.z = 3.0f;
    

    printf("%.3f\n",a.x);
    printf("%.3f\n",a.vec[0]);




    return 0;
}


