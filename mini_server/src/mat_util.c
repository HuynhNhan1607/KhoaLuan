#include "mat_util.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
// Matrix utility functions
void mat_copy(double *dst, const double *src, int rows, int cols) {
  memcpy(dst, src, sizeof(double) * rows * cols);
}

void mat_identity(double *A, int n) {
  memset(A, 0, sizeof(double) * n * n);
  for (int i = 0; i < n; ++i)
    A[i * n + i] = 1.0;
}

void mat_add(double *C, const double *A, const double *B, int rows, int cols) {
  for (int i = 0; i < rows * cols; ++i)
    C[i] = A[i] + B[i];
}

void mat_sub(double *C, const double *A, const double *B, int rows, int cols) {
  for (int i = 0; i < rows * cols; ++i)
    C[i] = A[i] - B[i];
}

void mat_mult(double *C, const double *A, const double *B, int m, int n,
              int p) {
  // C = A (m x n) * B (n x p) => C (m x p)
  for (int i = 0; i < m; ++i)
    for (int j = 0; j < p; ++j) {
      C[i * p + j] = 0.0;
      for (int k = 0; k < n; ++k)
        C[i * p + j] += A[i * n + k] * B[k * p + j];
    }
}

void mat_transpose(double *At, const double *A, int rows, int cols) {
  for (int i = 0; i < rows; ++i)
    for (int j = 0; j < cols; ++j)
      At[j * rows + i] = A[i * cols + j];
}

// Helper: 3x3 determinant for 4x4 cofactor calculation
static double det3x3(double a11, double a12, double a13, double a21, double a22,
                     double a23, double a31, double a32, double a33) {
  return a11 * (a22 * a33 - a23 * a32) - a12 * (a21 * a33 - a23 * a31) +
         a13 * (a21 * a32 - a22 * a31);
}

// Inverse for 1x1, 2x2, and 4x4 matrix
int mat_inv(double *inv, const double *A, int dim) {
  if (dim == 1) {
    if (fabs(A[0]) < 1e-9)
      return 0;
    inv[0] = 1.0 / A[0];
    return 1;
  }
  if (dim == 2) {
    double det = A[0] * A[3] - A[1] * A[2];
    if (fabs(det) < 1e-9)
      return 0;
    inv[0] = A[3] / det;
    inv[1] = -A[1] / det;
    inv[2] = -A[2] / det;
    inv[3] = A[0] / det;
    return 1;
  }
  if (dim == 4) {
    // 4x4 matrix inversion using cofactor/adjugate method
    // A is [a0  a1  a2  a3 ]
    //      [a4  a5  a6  a7 ]
    //      [a8  a9  a10 a11]
    //      [a12 a13 a14 a15]
    double a0 = A[0], a1 = A[1], a2 = A[2], a3 = A[3];
    double a4 = A[4], a5 = A[5], a6 = A[6], a7 = A[7];
    double a8 = A[8], a9 = A[9], a10 = A[10], a11 = A[11];
    double a12 = A[12], a13 = A[13], a14 = A[14], a15 = A[15];

    // Calculate cofactors (with alternating signs already applied)
    double c00 = det3x3(a5, a6, a7, a9, a10, a11, a13, a14, a15);
    double c01 = -det3x3(a4, a6, a7, a8, a10, a11, a12, a14, a15);
    double c02 = det3x3(a4, a5, a7, a8, a9, a11, a12, a13, a15);
    double c03 = -det3x3(a4, a5, a6, a8, a9, a10, a12, a13, a14);

    double c10 = -det3x3(a1, a2, a3, a9, a10, a11, a13, a14, a15);
    double c11 = det3x3(a0, a2, a3, a8, a10, a11, a12, a14, a15);
    double c12 = -det3x3(a0, a1, a3, a8, a9, a11, a12, a13, a15);
    double c13 = det3x3(a0, a1, a2, a8, a9, a10, a12, a13, a14);

    double c20 = det3x3(a1, a2, a3, a5, a6, a7, a13, a14, a15);
    double c21 = -det3x3(a0, a2, a3, a4, a6, a7, a12, a14, a15);
    double c22 = det3x3(a0, a1, a3, a4, a5, a7, a12, a13, a15);
    double c23 = -det3x3(a0, a1, a2, a4, a5, a6, a12, a13, a14);

    double c30 = -det3x3(a1, a2, a3, a5, a6, a7, a9, a10, a11);
    double c31 = det3x3(a0, a2, a3, a4, a6, a7, a8, a10, a11);
    double c32 = -det3x3(a0, a1, a3, a4, a5, a7, a8, a9, a11);
    double c33 = det3x3(a0, a1, a2, a4, a5, a6, a8, a9, a10);

    // Determinant = a0*c00 + a1*c01 + a2*c02 + a3*c03
    double det = a0 * c00 + a1 * c01 + a2 * c02 + a3 * c03;
    if (fabs(det) < 1e-12)
      return 0;

    double inv_det = 1.0 / det;

    // Inverse = adjugate / det (adjugate = transpose of cofactor matrix)
    inv[0] = c00 * inv_det;
    inv[1] = c10 * inv_det;
    inv[2] = c20 * inv_det;
    inv[3] = c30 * inv_det;
    inv[4] = c01 * inv_det;
    inv[5] = c11 * inv_det;
    inv[6] = c21 * inv_det;
    inv[7] = c31 * inv_det;
    inv[8] = c02 * inv_det;
    inv[9] = c12 * inv_det;
    inv[10] = c22 * inv_det;
    inv[11] = c32 * inv_det;
    inv[12] = c03 * inv_det;
    inv[13] = c13 * inv_det;
    inv[14] = c23 * inv_det;
    inv[15] = c33 * inv_det;
    return 1;
  }
  return 0; // Only support 1x1, 2x2, or 4x4
}