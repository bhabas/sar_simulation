#include <stdio.h>
#include <stdlib.h>
#include <float.h>
#include <math.h>
#include <stdarg.h>
#include <string.h>

int main()
{   
    typedef struct{
        double mean[3];
        double std[3];
    }Scaler;

    Scaler* Policy_Scaler;

    Policy_Scaler->mean[0] = 3.123;

    printf("Hello World %.3f\n",Policy_Scaler->mean[0]);
    
    char line[50];
    char *sp;
    float mean;
    float std;
    int studentId;


    FILE* fp = fopen("/home/bhabas/catkin_ws/src/crazyflie_simulation/crazyflie_gazebo/src/Info/Data_Test.csv", "r");
    if (fp == NULL) {
        perror("Error reading file\n");
        return 1;
    }

    // fgets(line,100,fp); // Skip line
    while(fgets(line,100,fp)!=NULL)
    {
        printf("%s\n",line);
        sp = strtok(line,",");
        studentId = atoi(sp);
        // mean = atof(sp);

        // printf("%f ",mean);
    }
   
    fclose(fp);

    return 0;
}