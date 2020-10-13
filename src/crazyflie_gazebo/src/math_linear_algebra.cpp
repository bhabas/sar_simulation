#include "math_linear_algebra.h"

namespace math {

double clamp(double value, double min, double max) {
  if (value < min) return min;
  if (value > max) return max;
  return value;
}
void hat(double *result, double *vector)
{
    // Pass in vector and get a skew-symmetric matrix representation
    double m[3][3] = {{0,0,0}, {0,0,0}, {0,0,0}};

    m[2][1] = vector[0];
    m[1][2] = -vector[0];
    m[0][2] = vector[1];
    m[2][0] = -vector[1];
    m[1][0] = vector[2];
    m[0][1] = -vector[2];

    std::memcpy(result, m, sizeof(m));
}

void dehat(double *result, double *matrix)
{
    // Pass in skew symmetric matrix and get a vector
    double vector[3];
    double tmp[3][3];

    std::memcpy(tmp, matrix, sizeof(tmp));

    // why double the value and take average and not just the first value?
    // a[0] = a1-(-a1)/2 vs a[0] = a1
    vector[0] = (tmp[2][1] - tmp[1][2])/2;
    vector[1] = (tmp[0][2] - tmp[2][0])/2;
    vector[2] = (tmp[1][0] - tmp[0][1])/2;

    std::memcpy(result, vector, sizeof(vector));
}

double dot(double *v1, double *v2, int n)
{
    double v = 0;
    for(int k=0;k<n;k++)
    {
        v += v1[k]*v2[k];
    }

    return v;
}

double matTranspose(double *result, double *matrix, int n)
{
    double m[n][n];
    double tmp[n][n];

    std::memcpy(tmp, matrix, sizeof(tmp));

    for(int i=0; i<n; i++)
    {
        for(int j=0; j<n; j++)
            m[i][j] = tmp[j][i];
    }

    std::memcpy(result, m, sizeof(m));
}

double matAddsMat(double *result, double *m1, double *m2, int size, int flag)
{
    double m[size];
    if (flag ==1)
    {
        for(int k=0;k<size;k++)
            m[k] = m1[k] + m2[k];       // addition
    }
    else if(flag ==2)
    {
        for(int k=0;k<size;k++)
            m[k] = m1[k] - m2[k];       // substraction
    }
    
    std::memcpy(result, m, sizeof(m));
}

double matTimesScalar(double *result, double *m1, double s, int size, int flag)
{
    double m[size];
    if (flag ==1)
    {
        for(int k=0;k<size;k++)
            m[k] = m1[k] * s;       // times
    }
    else if(flag ==2)
    {
        for(int k=0;k<size;k++)
            m[k] = m1[k] / s;       // divided by
    }
    
    std::memcpy(result, m, sizeof(m));
}

double matTimesVec(double *result, double *matrix, double *vector, int n)
{
    double v[n];
    double tmp1[n][n];

    std::memcpy(tmp1, matrix, sizeof(tmp1));

    for(int k=0; k<n; k++)
        v[k] = dot(tmp1[k], vector, n);

    std::memcpy(result, v, sizeof(v));
}

double matTimesMat(double *result, double *m1, double *m2)
{
    double m[3][3];
    double tmp1[3][3];
    double tmp2[3][3];

    std::memcpy(tmp1, m1, sizeof(tmp1));
    matTranspose((double *) tmp2, m2, 3);

    for(int i=0; i<3; i++)
    {
        for(int j=0; j<3; j++)
            m[i][j] = dot(tmp1[i], tmp2[j], 3);
    }

    std::memcpy(result, m, sizeof(m));
}

void quat2rotm_Rodrigue(double * result, double * quaternion)
{
    double rotm[3][3];
    double quat[4];

    std::memcpy(quat, quaternion, sizeof(quat));

    double quat_len = sqrt( dot(quat,quat,4) );
    //std::cout<<"quaternion is: ["<<quat[0]<<", "<<quat[1]<<", "<<quat[2]<<", "<<quat[3]<<"]"<<std::endl;
    //std::cout<<"quaternion norm: "<<quat_len<<std::endl;

    matTimesScalar(quat, quat, quat_len, 4, 2);
    // std::cout<<"quaternion normalized is: ["<<quat[0]<<", "<<quat[1]<<", "<<quat[2]<<", "<<quat[3]<<"]"<<std::endl;

    double q0 = quat[0];
    double q1 = quat[1];
    double q2 = quat[2];
    double q3 = quat[3];

    double theta = 2*acos(q0);
    double omega[3] = {0,0,0};
    double qvector[3] = {q1,q2,q3};
    if(theta!=0) //
        matTimesScalar(omega, qvector, sin(theta/2), 3, 2); // qvector*1/sin(0.5*theta)
    // std::cout << "Math QVec: " << quat[0] << " | " << quat[1] << " | "<< quat[2] << " | " << quat[3] << std::endl;
    // std::cout << "Math Angle: " << theta << std::endl;
    // std::cout << "Math: " << omega[0] << " | " << omega[1] << " | "<< omega[2] << std::endl << std::endl;
    double omega_hat[3][3] = { 
        {   0,         -omega[2],    omega[1]}, 
        { omega[2],     0,          -omega[0]}, 
        {-omega[1],     omega[0],    0       }};

    double tmp1[3][3];
    double tmp2[3][3];
    double tmp3[3][3];
    double tmp4[3][3];

    double I[3][3] = { // Identity matrix
        {1,0,0}, 
        {0,1,0}, 
        {0,0,1}};

        
    // rotm = I(3x3) + sin(theta)*omega_hat(3x3) + (1-cos(theta))*omega_hat^2 (3x3)
    matTimesScalar((double *) tmp1,(double *) omega_hat, sin(theta), 3*3, 1); // sin(theta)*omega_hat(3x3)
    matTimesMat((double *)tmp2, (double *)omega_hat, (double *)omega_hat); // omega_hat(3x3)*omega_hat(3x3)
    matTimesScalar((double *)tmp3, (double *)tmp2, 1-cos(theta), 3*3, 1); // (1-cos(theta))*omega_hat^2 (3x3)
    matAddsMat((double *) tmp4,(double *) I,(double *) tmp1, 3*3, 1); // I(3x3) + sin(theta)*omega_hat(3x3)
    matAddsMat((double *) rotm,(double *) tmp4,(double *) tmp3, 3*3, 1); 
    
    
    std::memcpy(result, rotm, sizeof(rotm));
    
}


}