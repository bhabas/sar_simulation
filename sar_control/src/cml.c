#include "cml.h"

cml_m33 cml_m33_new()
{
    cml_m33 m;
    m.num_rows = 3;
    m.num_cols = 3;
    memset(m.data, 0, sizeof(m.data));  // Initialize all data to 0
    return m;
}

cml_m31 cml_m31_new()
{
    cml_m31 m;
    m.num_rows = 3;
    m.num_cols = 1;
    memset(m.data, 0, sizeof(m.data));  // Initialize all data to 0
    return m;
}


void cml_mat_print(void* m, int rows, int cols)
{
    if(m == NULL) {
        printf("Null matrix pointer!\n");
        return;
    }
    
    // Cast to a float pointer for access
    float (*data)[cols] = m;
    
    for (int i = 0; i < rows; i++)
    {
        for (int j = 0; j < cols; j++)
        {
            printf("%.5f ", data[i][j]);
        }
        printf("\n");
    }
}
