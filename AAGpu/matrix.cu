#include "matrix.cuh"

__device__ __host__ matrix<2, 1> operator/(const matrix<2, 2> &A, const matrix<2, 1> &b)
{
    float y = (b(1, 0) - A(1, 0) * b(0, 0) / A(0, 0)) /
        (A(1, 1) - A(1, 0) * A(0, 1) / A(0, 0));
    float x = (b(0, 0) - A(0, 1) * y) / A(0, 0);
    return colVec(x, y);
}