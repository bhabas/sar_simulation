#include <stdio.h>
#include <string.h>
#include <stdlib.h>


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

    


int main()
{   
    // EXTRACT MATRIX DIMENSIONS

    char array_list[1024], array_str[1024], line_str[1024];
    char *save_ptr1, *save_ptr2, *save_ptr3;
    char *array_token, *line_token, *value_token;

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

    


    // // get the initial tokens
    // token1 = strtok_r(buf_1, " ", &save_ptr1);
    // token2 = strtok_r(buf_2, " ", &save_ptr2);
    // token3 = strtok_r(buf_3, " ", &save_ptr3);

    // while(token1 && token2 && token3) {
    //     // do stuff with tokens

    //     // get next tokens
    //     token1 = strtok_r(NULL, " ", &save_ptr1);
    //     token2 = strtok_r(NULL, " ", &save_ptr2);
    //     token3 = strtok_r(NULL, " ", &save_ptr3);
    // }


    // // EXTRACT ARRAYS
    // char* array_token = strtok(str, "*");
    // while(array_token!= NULL){

    //     char* array_str;
    //     strcpy(array_str,array_token);
    //     // char* line_token = strtok(array_token,",");
    //     printf("%s\n",array_str);



    //     // printf("m: %d \t n: %d\n",m,n);





    
    //     array_token = strtok(NULL, "*");
    // }


    // char* token = strtok(token_Line, " ");





    

    return 0;
}


