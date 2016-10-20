#pragma once

#include "cuda_runtime.h"
#include "device_launch_parameters.h"

#include <stdexcept>
#include <string>

// Wrappers for cuda device functions

template <typename F, typename ...T> 
void cuwrap(F f, T... args)
{
    cudaError st = f(args...);
    if (st != cudaSuccess)
        throw std::runtime_error("Cuda error calling");
}

template <typename T> struct ddpointer
{
    typedef T type;
};

template <typename T> struct ddpointer<T*>
{
    typedef typename ddpointer<T>::type type;
};

template <typename T>
void massalloc(unsigned count, T ptr)
{
    size_t tsize = sizeof(ddpointer<T>::type);
    cudaError st = cudaMalloc(ptr, count * tsize);
    if (st != cudaSuccess)
        throw std::runtime_error("Malloc error");
}

template <typename T, typename ...Ts>
void massalloc(unsigned count, T ptr, Ts... args)
{
    size_t tsize = sizeof(ddpointer<T>::type);
    cudaError st = cudaMalloc(ptr, count * tsize);
    if (st != cudaSuccess)
        throw std::runtime_error("Malloc error");
    massalloc(count, args...);
}

template <typename T>
void massalloc_cpu(unsigned count, T **ptr)
{
    size_t tsize = sizeof(T);
    *ptr = (T*)malloc(count * tsize);
}

template <typename T, typename ...Ts>
void massalloc_cpu(unsigned count, T **ptr, Ts**... args)
{
    size_t tsize = sizeof(T);
    *ptr = (T*)malloc(count * tsize);
    massalloc_cpu(count, args...);
}

template <typename T>
void copyGpuCpu(T *gpu, T *host, unsigned count)
{
    size_t tsize = sizeof(T) * count;
    cudaError st = cudaMemcpy(host, gpu, tsize, cudaMemcpyDeviceToHost);
    if (st != cudaSuccess)
        throw std::runtime_error("GPU-CPU transfer error");
}

template <typename T>
void copyCpuGpu(T *host, T *gpu, unsigned count)
{
    size_t tsize = sizeof(T) * count;
    cudaError st = cudaMemcpy(gpu, host, tsize, cudaMemcpyHostToDevice);
    if (st != cudaSuccess)
        throw std::runtime_error("CPU-GPU transfer error");
}

template <typename T>
void copyCpuCpu(T *cpu, T *host, unsigned count)
{
    size_t tsize = sizeof(T) * count;
    memcpy(host, cpu, tsize);
}

template <typename T>
void massfree(T ptr)
{
    cudaError st = cudaFree(ptr);
    if (st != cudaSuccess)
        throw std::runtime_error("Free error");
}

template <typename T, typename ...Ts>
void massfree(T ptr, Ts... args)
{
    cudaError st = cudaFree(ptr);
    if (st != cudaSuccess)
        throw std::runtime_error("Free error");
    massfree(args...);
}

template <typename T>
void massfree_cpu(T ptr)
{
    free(ptr);
}

template <typename T, typename ...Ts>
void massfree_cpu(T ptr, Ts... args)
{
    free(ptr);
    massfree_cpu(args...);
}