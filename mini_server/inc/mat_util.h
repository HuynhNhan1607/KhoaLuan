#ifndef MAT_UTIL_H_
#define MAT_UTIL_H_

void mat_copy(double *dst, const double *src, int rows, int cols);
void mat_identity(double *A, int n);
void mat_add(double *C, const double *A, const double *B, int rows, int cols);
void mat_sub(double *C, const double *A, const double *B, int rows, int cols);
void mat_mult(double *C, const double *A, const double *B, int m, int n, int p);
void mat_transpose(double *At, const double *A, int rows, int cols);
int mat_inv(double *inv, const double *A, int dim);

#endif