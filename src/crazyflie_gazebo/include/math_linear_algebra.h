#include <iostream>
#include <cmath>
#include <cstring>      // provides memcpy

namespace math {

void hat(double *result, double *vector);

void dehat(double *result, double *matrix);

double dot(double *v1, double *v2, int n);

double matTranspose(double *result, double *matrix, int n);

double matAddsMat(double *result, double *m1, double *m2, int size, int flag);

double matTimesScalar(double *result, double *m1, double s, int size, int flag);

double matTimesVec(double *result, double *matrix, double *vector, int n);

double matTimesMat(double *result, double *m1, double *m2);

void quat2rotm_Rodrigue(double * result, double * quaternion);


}