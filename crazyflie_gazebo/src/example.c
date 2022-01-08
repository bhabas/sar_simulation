#include <stdio.h>
#include <stdlib.h>
#include <float.h>
#include <math.h>
#include <stdarg.h>
#include <string.h>

typedef struct{
    float mean[3];
    float std[3];
}Scaler;

int readCSV(Scaler* scaler)
{

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
        sp = strtok(line,",");
        scaler->mean[i] = atof(sp);

        sp = strtok(NULL,",");
        scaler->std[i] = atof(sp);
    
    }

    printf("%.3f\n",scaler->mean[i]);
    fclose(fp);
    
    return 0;
}

int main()
{   
    

    Scaler Policy_Scaler;

    readCSV(&Policy_Scaler);

    

    return 0;
}


