
#include "cuda_runtime.h"
#include "device_launch_parameters.h"
#include "vector_functions.hpp"

#include <stdio.h>

#include "AAGpu.hpp"
#include "aircraftmodel.cuh"

cudaError_t run_simulation();

__global__ void model_kernel(float *aoa_output, float start_vel, float moi, float3 rot_m, 
    float3 lift_m, float2 drag_m, float mass, float sas, float dt, int length)
{
    __shared__ pitch_model model;
    // initialize model
    model.velocity.x = start_vel;
    model.moi = moi;
    model.rot_m = rot_m;
    model.lift_m = lift_m;
    model.drag_m = drag_m;
    model.sas_torque = sas;
    model.mass = mass;

    // simulate
    aoa_output[0] = model.aoa;
    for (int i = 0; i < length; i++)
    {
        model.preupdate(dt);
        model.simulation_step(dt, 0.0f);
        aoa_output[i + 1] = model.aoa;
    }
}

namespace AAGpu
{

    int execute()
    {
        cudaError_t cudaStatus = run_simulation();
        if (cudaStatus != cudaSuccess)
        {
            fprintf(stderr, "addWithCuda failed!");
            return 1;
        }

        // cudaDeviceReset must be called before exiting in order for profiling and
        // tracing tools such as Nsight and Visual Profiler to show complete traces.
        cudaStatus = cudaDeviceReset();
        if (cudaStatus != cudaSuccess)
        {
            fprintf(stderr, "cudaDeviceReset failed!");
            return 1;
        }

        return 0;
    }

}

// Helper function for using CUDA to add vectors in parallel.
cudaError_t run_simulation()
{
    cudaError_t cudaStatus;

    float aoas[101];
    float *d_aoas;

    // Choose which GPU to run on, change this on a multi-GPU system.
    cudaStatus = cudaSetDevice(0);
    if (cudaStatus != cudaSuccess) 
    {
        fprintf(stderr, "cudaSetDevice failed!  Do you have a CUDA-capable GPU installed?");
        goto Error;
    }

    cudaStatus = cudaMalloc(&d_aoas, 101 * sizeof(float));
    if (cudaStatus != cudaSuccess)
    {
        fprintf(stderr, "cudaMalloc failed!");
        goto Error;
    }

    // Launch a kernel on the GPU with one thread for each element.
    model_kernel<<<1, 1>>>(d_aoas, 200.0f, 165.0f, make_float3(0.0f, -1.0f, 1.15f),
        make_float3(0.0f, 60.0f, -0.25f), make_float2(1.0f, 20.0f), 14.0f, 
        15.0f, 0.05f, 100);

    // Check for any errors launching the kernel
    cudaStatus = cudaGetLastError();
    if (cudaStatus != cudaSuccess) 
    {
        fprintf(stderr, "model_kernel launch failed: %s\n", cudaGetErrorString(cudaStatus));
        goto Error;
    }
    
    // cudaDeviceSynchronize waits for the kernel to finish, and returns
    // any errors encountered during the launch.
    cudaStatus = cudaDeviceSynchronize();
    if (cudaStatus != cudaSuccess) 
    {
        fprintf(stderr, "cudaDeviceSynchronize returned error code %d after launching addKernel!\n", cudaStatus);
        goto Error;
    }

    cudaStatus = cudaMemcpy(aoas, d_aoas, 101 * sizeof(float), cudaMemcpyDeviceToHost);
    if (cudaStatus != cudaSuccess)
    {
        fprintf(stderr, "cudaMemcpy failed!");
        goto Error;
    }

    cudaStatus = cudaFree(d_aoas);
    if (cudaStatus != cudaSuccess)
    {
        fprintf(stderr, "cudaFree failed!");
        goto Error;
    }

    puts("AoA's:");
    for (int i = 0; i < 101; i++)
    {
        printf("%f ", aoas[i]);
        if (i == 100)
            printf("\n");
    }

    auto file = fopen("simul.csv", "w");
    for (int i = 1; i < 101; i++)
    {
        if (i < 100)
            fprintf(file, "%f, ", aoas[i]);
        else
            fprintf(file, "%f", aoas[i]);
    }

Error:    
    return cudaStatus;
}
