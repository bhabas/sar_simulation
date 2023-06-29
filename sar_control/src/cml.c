#include "cml.h"

void cml_m33_init(cml_m33* m)
{
    m->num_rows = 3;
    m->num_cols = 3;
    memset(m->data, 0, sizeof(m->data));
}


void cml_m31_init(cml_m31* m)
{
    m->num_rows = 3;
    m->num_cols = 1;
    memset(m->data, 0, sizeof(m->data));
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
