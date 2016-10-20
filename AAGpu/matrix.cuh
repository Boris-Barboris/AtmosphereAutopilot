#pragma once

#include "cuda_runtime.h"

template <unsigned Row, unsigned Rows, unsigned Cols> struct matrixrow;

template <unsigned Rows, unsigned Cols> struct matrix
{
    float data[Rows * Cols];

    inline __device__ __host__ float& operator()(unsigned row, unsigned column)
    {
        return data[Cols * row + column];
    }

    inline __device__ __host__ float operator()(unsigned row, unsigned column) const
    {
        return data[Cols * row + column];
    }

    inline __device__ __host__ float& get(unsigned row, unsigned column)
    {
        return data[Cols * row + column];
    }

    inline __device__ __host__ float getc(unsigned row, unsigned column) const
    {
        return data[Cols * row + column];
    }

    template <unsigned R, unsigned C>
    __device__ __host__ void copyFrom(const matrix<R, C> &mat)
    {
        static_assert((R >= Rows) && (C >= Cols), "Can only copy from the matrix larger than current");

        for (int i = 0; i < Rows; i++)
            for (int j = 0; j < Cols; j++)
                get(i, j) = mat.getc(i, j);
    }

    template <unsigned R>
    __device__ __host__ matrix<Rows, R> operator*(const matrix<Cols, R> &rhs) const
    {
        matrix<Rows, R> res;
        for (int r = 0; r < Rows; r++)
            for (int c = 0; c < R; c++)
            {
                float val = 0.0f;
                for (int k = 0; k < Cols; k++)
                    val += getc(r, k) * rhs.getc(k, c);
                res(r, c) = val;
            }
        return res;
    }

    __device__ __host__ matrix<Rows, Cols> scale(const matrix<Rows, Cols> &rhs) const
    {
        matrix<Rows, Cols> res;
        for (int r = 0; r < Rows; r++)
            for (int c = 0; c < Cols; c++)
                res(r, c) = rhs(r, c) * getc(r, c);
        return res;
    }

    __device__ __host__ matrix<Rows, Cols> operator*(float rhs) const
    {
        matrix<Rows, Cols> res;
        for (int r = 0; r < Rows; r++)
            for (int c = 0; c < Cols; c++)
                res(r, c) = rhs * getc(r, c);
        return res;
    }

    __device__ __host__ matrix<Rows, Cols> operator+(float rhs) const
    {
        matrix<Rows, Cols> res;
        for (int r = 0; r < Rows; r++)
            for (int c = 0; c < Cols; c++)
                res(r, c) = rhs + getc(r, c);
        return res;
    }

    __device__ __host__ matrix<Rows, Cols> operator+(const matrix<Rows, Cols> &rhs) const
    {
        matrix<Rows, Cols> res;
        for (int r = 0; r < Rows; r++)
            for (int c = 0; c < Cols; c++)
                res(r, c) = rhs.getc(r, c) + getc(r, c);
        return res;
    }

    __device__ __host__ matrix<Rows, Cols> operator-(const matrix<Rows, Cols> &rhs) const
    {
        matrix<Rows, Cols> res;
        for (int r = 0; r < Rows; r++)
            for (int c = 0; c < Cols; c++)
                res(r, c) = getc(r, c) - rhs.getc(r, c);
        return res;
    }

    __device__ __host__ matrix<Rows, Cols> operator-(float rhs) const
    {
        matrix<Rows, Cols> res;
        for (int r = 0; r < Rows; r++)
            for (int c = 0; c < Cols; c++)
                res(r, c) = getc(r, c) - rhs;
        return res;
    }

    inline __device__ __host__ explicit operator float() const
    {
        static_assert((Rows == 1) && (Cols == 1), "We're not matrix-element");

        return data[0];
    }

    template <unsigned R>
    inline __device__ __host__ matrixrow<R, Rows, Cols> rowSlice()
    {
        static_assert(R < Rows, "No such row");

        matrixrow<R, Rows, Cols> res(*this);
        return res;
    }
};

template <unsigned Row, unsigned Rows, unsigned Cols> struct matrixrow
{
    matrix<Rows, Cols> &hdl;

    __device__ __host__ matrixrow(matrix<Rows, Cols> &host) : hdl(host) {}

    inline __device__ __host__ float& operator()(unsigned row, unsigned column)
    {
        return hdl.data[Row * Cols + column];
    }

    inline __device__ __host__ float operator()(unsigned row, unsigned column) const
    {
        return hdl.data[Row * Cols + column];
    }

    inline __device__ __host__ float& get(unsigned row, unsigned column)
    {
        return hdl.data[Row * Cols + column];
    }

    inline __device__ __host__ float getc(unsigned row, unsigned column) const
    {
        return hdl.data[Row * Cols + column];
    }

    template <unsigned R>
    __device__ __host__ matrix<1, R> operator*(const matrix<Cols, R> &rhs) const
    {
        matrix<1, R> res;
        for (int c = 0; c < R; c++)
        {
            float val = 0.0f;
            for (int k = 0; k < Cols; k++)
                val += getc(0, k) * rhs.getc(k, c);
            res(0, c) = val;
        }
        return res;
    }

    __device__ __host__ matrix<1, Cols> operator*(float rhs) const
    {
        matrix<1, Cols> res;
        for (int c = 0; c < Cols; c++)
            res(0, c) = rhs * getc(0, c);
        return res;
    }

    __device__ __host__ matrix<1, Cols> operator+(float rhs) const
    {
        matrix<1, Cols> res;
        for (int c = 0; c < Cols; c++)
            res(0, c) = rhs + getc(0, c);
        return res;
    }

    __device__ __host__ matrix<1, Cols> operator+(const matrix<1, Cols> &rhs) const
    {
        matrix<1, Cols> res;
        for (int c = 0; c < Cols; c++)
            res(0, c) = rhs.getc(0, c) + getc(0, c);
        return res;
    }

    __device__ __host__ matrix<1, Cols> operator-(float rhs) const
    {
        matrix<1, Cols> res;
        for (int c = 0; c < Cols; c++)
            res(0, c) = getc(0, c) - rhs;
        return res;
    }

    __device__ __host__ matrix<1, Cols> operator-(const matrix<1, Cols> &rhs) const
    {
        matrix<1, Cols> res;
        for (int c = 0; c < Cols; c++)
            res(0, c) = getc(0, c) - rhs.getc(0, c);
        return res;
    }

    inline __device__ __host__ explicit operator float() const
    {
        static_assert((Cols == 1), "We're not matrix-element");

        return getc(0, 0);
    }
};


template <int N, int Size, typename T1, typename ...T>
__device__ __host__ void __colVec(matrix<Size, 1> &mat, T1 v1, T... args)
{
    mat(N, 0) = v1;
    __colVec<N+1, Size, T...>(mat, args...);
}

template <int N, int Size, typename T1>
__device__ __host__ void __colVec(matrix<Size, 1> &mat, T1 v1)
{
    mat(N, 0) = v1;
}

template <typename ...T> 
__device__ __host__ matrix<sizeof...(T), 1> colVec(T... args)
{
    const int size = sizeof...(T);
    matrix<size, 1> res;
    __colVec<0, size, T...>(res, args...);
    return res;
}


// solve A*x=b with nonzero A(0, 0)
__device__ __host__ matrix<2, 1> operator/(const matrix<2, 2> &A, const matrix<2, 1> &b);