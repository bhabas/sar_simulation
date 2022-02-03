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
    "1.1  2,"
    "3.4  4,"
    "5.653  6,"
    "*"
    "3 3,"
    "1  2  3,"
    "4  5  6,"
    "7  8  9,"

};


nml_mat* nml_mat_fromstr(char* str)
{
    unsigned int num_rows = 0, num_cols = 0;
    char array_str[1024],line_str[1024];
    char *line_token, *value_token;
    char *save_ptr1, *save_ptr2;

    strcpy(array_str,str);
    line_token = strtok_r(array_str,",",&save_ptr1);

    // DETERMINE DIMENSIONS FROM FIRST LINE
    value_token = strtok_r(line_token," ",&save_ptr2); // Collect first value of line
    num_rows = atoi(value_token);

    value_token = strtok_r(NULL," ",&save_ptr2); // Collect second value of line
    num_cols = atoi(value_token);
    nml_mat* r = nml_mat_new(num_rows,num_cols);

    // ITERATE THROUGH LINES
    line_token = strtok_r(NULL,",",&save_ptr1); // Move to next line
    for (int i = 0; i < num_rows; i++)
    {

        value_token = strtok_r(line_token," ",&save_ptr2);
        for (int j = 0; j < num_cols; j++)
        {
            r->data[i][j] = atof(value_token);
            value_token = strtok_r(NULL," ",&save_ptr2);
        }
        
        line_token = strtok_r(NULL,",",&save_ptr1);
    }
    


    return r;


}


int main()
{   
    nml_mat* X;

    X = nml_mat_fromstr(str);
    nml_mat_print(X);

    

    return 0;
}


