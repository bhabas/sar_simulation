#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "nml.h"

// Compile Statement
// gcc example.c nml.c nml_util.c -I /home/bhabas/catkin_ws/src/crazyflie_simulation/crazyflie_gazebo/include  -Wall -o example -lm -Wall && ./example 

/*
Seperate arrays into strings grouped by "*"
Seperate string by "," to get first line
Seperate first line by " " to get dimensions
Allocate matrix by dimensions
Loop through rows
    Loop through columns
        Add value to matrix

*/


static char str[] = {
    "3 2,"
    "1  2,"
    "3  4,"
    "5  6,"
    "*"
    "3 3,"
    "1  2  3,"
    "4  5  6,"
    "7  8  9,"

};


nml_mat* nml_mat_fromstr(char* array_str)
{
    unsigned int num_rows = 0, num_cols = 0;

}


int main()
{   


    // EXTRACT MATRIX DIMENSIONS

    char array_list[5000], array_str[1024], line_str[1024];
    char *array_token, *line_token, *value_token;
    char *save_ptr1, *save_ptr2, *save_ptr3;

    strcpy(array_list,str);


    // ITERATE THROUGH ARRAYS
    array_token = strtok_r(array_list,"*",&save_ptr1); // Collect first array
    while(array_token!=NULL)
    {   

        line_token = strtok_r(array_token,",",&save_ptr2); // Collect first line
        printf("%s\n",line_token);

        
        // DETERMINE DIMENSIONS FROM FIRST LINE
        value_token = strtok_r(line_token," ",&save_ptr3); // Collect first value from first line
        int m = atoi(value_token);

        value_token = strtok_r(NULL," ",&save_ptr3); // Collect second value from first line
        int n = atoi(value_token);
        printf("m: %d \t n: %d\n",m,n);


        

        // ITERATE THROUGH LINES
        line_token = strtok_r(NULL,",",&save_ptr2); // Move to next line
        for (int i = 0; i < m; i++)
        {
            printf("%s\n",line_token);

            value_token = strtok_r(line_token," ",&save_ptr3); // Collect first value from first line
            for (int j = 0; j < n; j++)
            {
                printf("%s\n",value_token);
                value_token = strtok_r(NULL," ",&save_ptr3); // Move to next value
            }
            printf("\n");
            

            line_token = strtok_r(NULL,",",&save_ptr2);
        }
        

        
        array_token = strtok_r(NULL, "*", &save_ptr1);

    }

    
    

    return 0;
}


