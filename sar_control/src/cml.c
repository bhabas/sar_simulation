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

void cml_mat_get_column(void* m, int m_rows, int m_cols, int col_get, float output[])
{

    if (m == NULL) {
        printf("Null matrix pointer!\n");
        return;
    }
    if (col_get < 0 || col_get >= m_cols) {
        printf("Column index out of bounds!\n");
        return;
    }

    // Cast to a float pointer for access
    float (*data)[m_cols] = m;

    for (int i = 0; i < m_rows; i++) {
        output[i] = data[i][col_get];
    }
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
