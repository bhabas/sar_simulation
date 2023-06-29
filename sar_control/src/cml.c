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

void cml_mat_fill_fromarr(void* m, int m_rows, int m_cols, float arr[], int arr_size)
{
    if (m == NULL) {
        printf("Null matrix pointer!\n");
        return;
    }
    if (arr == NULL) {
        printf("Null array pointer!\n");
        return;
    }
    if (arr_size != m_rows * m_cols) {
        printf("Array size does not match matrix size!\n");
        return;
    }

    // Cast to a float pointer for access
    float (*data)[m_cols] = m;

    int index = 0;
    for (int i = 0; i < m_rows; i++) {
        for (int j = 0; j < m_cols; j++) {
            data[i][j] = arr[index++];
        }
    }
}

cml_m31 cml_m31_col_get(cml_m33* m, int col) 
{
    cml_m31 v;
    cml_m31_init(&v);

    if (m == NULL) {
        printf("Null matrix pointer!\n");
        return v; // return an empty vector
    }
    if (col < 0 || col >= m->num_cols) {
        printf("Column index out of bounds!\n");
        return v; // return an empty vector
    }

    for (int i = 0; i < v.num_rows; i++) {
        v.data[i][0] = m->data[i][col];
    }

    return v; // return the filled vector
}


void cml_mat_print(void* m, int m_rows, int m_cols)
{
    if(m == NULL) {
        printf("Null matrix pointer!\n");
        return;
    }
    
    // Cast to a float pointer for access
    float (*data)[m_cols] = m;
    
    for (int i = 0; i < m_rows; i++)
    {
        for (int j = 0; j < m_cols; j++)
        {
            printf("%.5f ", data[i][j]);
        }
        printf("\n");
    }
}
