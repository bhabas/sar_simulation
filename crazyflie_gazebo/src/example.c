#include <stdio.h>
#include <stdlib.h>
#include <float.h>
#include <math.h>
#include <stdarg.h>
#include <string.h>

int main()
{   
    typedef struct{
        float mean[3];
        float std[3];
    }Scaler;

    Scaler Policy_Scaler;

    char line[50];
    char *sp;


    FILE* fp = fopen("/home/bhabas/catkin_ws/src/crazyflie_simulation/crazyflie_gazebo/src/Info/Scaler_Policy_Value.csv", "r");
    if (fp == NULL) {
        perror("Error reading file\n");
        return 1;
    }
    
    fgets(line,100,fp); // Skip buffer
    int i = 0;
    while(fgets(line,100,fp)!=NULL)
    {
        sp = strtok(line,"\t");
        Policy_Scaler.mean[i] = atof(sp);

        sp = strtok(NULL,"\t");
        Policy_Scaler.std[i] = atof(sp);
    
    }

    printf("%.3f\n",Policy_Scaler.mean[i]);
    fclose(fp);

    return 0;
}