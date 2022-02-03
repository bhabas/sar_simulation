#include <example.h>

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

int main()
{   
    nml_mat* X1;
    
    char array_list[2048];
    char* array_token;
    char* save_ptr;

    strcpy(array_list,str);

    array_token = strtok_r(array_list,"*",&save_ptr);
    
    while(array_token!=NULL)
    {
        printf("%s\n",array_token);
        X1 = nml_mat_fromstr(array_token);
        nml_mat_print(X1);
        array_token = strtok_r(NULL,"*",&save_ptr);
    }

    
    

    

    return 0;
}


