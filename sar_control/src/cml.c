#include "cml.h"

void cml_m33_init(cml_m33* m)
{
    m->num_rows = 3;
    m->num_cols = 3;
    memset(m->data, 0, sizeof(m->data));
}

cml_m33 cml_m33_new()
{
    cml_m33 m;
    m.num_rows = 3;
    m.num_cols = 3;
    memset(m.data, 0, sizeof(m.data));

    return m;
}

cml_m33 cml_m33_cp(cml_m33* m)
{
    cml_m33 r = cml_m33_new();

    for (int i = 0; i < r.num_rows; i++)
    {
        for (int j = 0; j < r.num_cols; j++)
        {
            r.data[i][j] = m->data[i][j];
        }
        
    }
    return r;
}

cml_m33 cml_m33_eye()
{
    cml_m33 m = cml_m33_new();

    for (int i = 0; i < 3; i++)
    {
        m.data[i][i] = 1.0f;
    }
    return m;
}


void cml_m31_init(cml_m31* m)
{
    m->num_rows = 3;
    m->num_cols = 1;
    memset(m->data, 0, sizeof(m->data));
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
    printf("\n");
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



// Sets all elements of the matrix to given value
int cml_m33_diag_set(cml_m33 *m, double value)
{

  for(int i = 0; i < m->num_rows; i++) {
    m->data[i][i] = value;
  }
  return 1;
}

void cml_m33_row_swap_r(cml_m33* m, int row1, int row2)
{
    if (m == NULL) {
        printf("Null matrix pointer!\n");
        return;
    }
    if (row1 < 0 || row1 >= m->num_rows || row2 < 0 || row2 >= m->num_rows) {
        printf("Row index out of bounds!\n");
        return;
    }

    // Swap rows using a temporary array
    float temp[3];
    memcpy(temp, m->data[row1], sizeof(temp));
    memcpy(m->data[row1], m->data[row2], sizeof(temp));
    memcpy(m->data[row2], temp, sizeof(temp));
}

void cml_m33_col_swap_r(cml_m33* m, int col1, int col2)
{

    if (m == NULL) {
        printf("Null matrix pointer!\n");
        return;
    }
    if (col1 < 0 || col1 >= m->num_cols || col2 < 0 || col2 >= m->num_cols) {
        printf("Column index out of bounds!\n");
        return;
    }

    for (int i = 0; i < m->num_rows; i++) {
        float temp = m->data[i][col1];
        m->data[i][col1] = m->data[i][col2];
        m->data[i][col2] = temp;
    }
}


void cml_m33_row_addrow_r(cml_m33* m, int where, int row, float multiplier)
{
    if (m == NULL || where >= m->num_rows || row >= m->num_rows) {
        printf("Invalid matrix or row index!\n");
        return;
    }

    for (int i = 0; i < m->num_cols; i++) {
        m->data[where][i] += multiplier * m->data[row][i];
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

// Useful for QR decomposition
// Represents the (dot) product of two vectors:
// vector1 = m1col column from m1
// vector2 = m2col column from m2
float cml_vec_dot(cml_m33* m1, int m1_col, cml_m33* m2, int m2_col)
{
    if (m1->num_rows!=m2->num_rows) {
        printf("CANNOT_VECT_DOT_DIMENSIONS");
    }
    if (m1_col >= m1->num_cols) {
        printf("CANNOT_GET_COLUMN");
    }
    if (m2_col >= m2->num_cols) {
        printf("CANNOT_GET_COLUMN");
    }

    int i;
    float dot = 0.0;
    for(i = 0; i < m1->num_rows; i++) {
        dot += m1->data[i][m1_col] * m2->data[i][m1_col];
    }
    return dot;

}


// *****************************************************************************
//
// LUP Decomposition
//
// *****************************************************************************

// Finds the maxid on the column (starting from k -> num_rows)
// This method is used for pivoting in LUP decomposition
int cml_m33_absmaxr(cml_m33* m, int k)
{
    if (m == NULL || k < 0 || k >= m->num_cols) {
        printf("Invalid matrix or column index!\n");
        return -1; // Return an invalid index as an error indicator
    }

    int maxIdx = k;
    float maxVal = fabs(m->data[k][k]);

    for (int i = k + 1; i < m->num_rows; i++) {
        if (fabs(m->data[i][k]) > maxVal) {
            maxVal = fabs(m->data[i][k]);
            maxIdx = i;
        }
    }

    return maxIdx;
}

// Creates a new nml_mat_lup structure
cml_m33_lup cml_m33_lup_new(cml_m33 L, cml_m33 U, cml_m33 P, int num_permutations)
{
    cml_m33_lup r;

    r.L = L;
    r.U = U;
    r.P = P;
    r.num_permutations = num_permutations;

  return r;
}

void cml_m33_lup_print(cml_m33_lup *lup) 
{
  cml_mat_print(&(lup->L),3,3);
  cml_mat_print(&(lup->U),3,3);
  cml_mat_print(&(lup->P),3,3);
}

cml_m33_lup cml_m33_lup_solve(cml_m33* m)
{
    cml_m33 L = cml_m33_new();
    cml_m33 U = cml_m33_cp(m);
    cml_m33 P = cml_m33_eye();

    // cml_mat_print(&P,3,3);
    int num_permutations = 0;

    int pivot;
    float mult;

    for (int j = 0; j < U.num_cols; j++)
    {
        // Retrieves the row with the biggest element for column (j)
        pivot = cml_m33_absmaxr(&U,j);
        if (fabs(U.data[pivot][j]) < CML_MIN_COEF)
        {
            printf("CML Error: CANNOT LU MATRIX DEGENERATE");
        }
        
        if (pivot!=j)
        {
            // Pivots LU and P accordingly to the rule
            cml_m33_row_swap_r(&L, j, pivot);
            cml_m33_row_swap_r(&U, j, pivot);
            cml_m33_row_swap_r(&P, j, pivot);

            // Keep the number of permutations to easiliy calculate the
            // determinant sign afterwards
            num_permutations++;
        }
        for (int i = j+1; i < U.num_rows; i++)
        {
            mult = U.data[i][j] / U.data[j][j];
            // Building the U uppper rows
            cml_m33_row_addrow_r(&U, i, j, -mult);
            // Store the multiplier in L
            L.data[i][j] = mult;
        }
    }
    cml_m33_diag_set(&L,1.0);

    cml_m33_lup LUP = cml_m33_lup_new(L,U,P,num_permutations);
    
    return LUP;
}






    