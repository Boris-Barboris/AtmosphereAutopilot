#include "AAGpu.h"

#include "cuda_wrappers.cuh"
#include "ang_vel_pitch.cuh"
#include "ang_acc_pitch.cuh"


struct thread_context
{
    pitch_model mdl;
    ang_vel_p vel_c;
};

#define RAWPREFIX __global__
#define RAWFUNCNAME raw_kernel
#define RAWCONTEXTPREFIX __shared__

#include "raw_kernel_common.cu"

#define RAWPREFIX __host__
#define RAWFUNCNAME raw_kernel_cpu
#define RAWCONTEXTPREFIX 

#include "raw_kernel_common.cu"


void raw_execute(
    float dt,
    int step_count,
    float moi,
    float mass,
    float sas,
    const std::array<float, 3> &rot_model,
    const std::array<float, 3> &lift_model,
    const std::array<float, 2> &drag_model,
    bool aero_model_par,
    float start_vel,
    bool keep_speed,
    float input,
    std::vector<float> &out_angvel,
    std::vector<float> &out_aoa,
    std::vector<float> &out_acc,
    std::vector<float> &out_csurf,
    std::vector<float> &out_input)
{
    float *d_angvel, *d_aoa, *d_acc, *d_csurf, *d_input;

    cuwrap(cudaSetDevice, 0);
    massalloc(step_count + 1, &d_angvel, &d_aoa, &d_acc, &d_csurf, &d_input);

    //cudaError r;
    cudaMemcpyToSymbol(d_aero_model, &aero_model_par, sizeof(bool));
    cudaMemcpyToSymbol(d_spd_const, &keep_speed, sizeof(bool));

    raw_kernel<<<1, 1>>>(
        d_angvel,
        d_aoa,
        d_acc,
        d_csurf,
        d_input,
        dt,
        step_count,
        moi,
        mass,
        sas,
        input,
        make_float3(rot_model[0], rot_model[1], rot_model[2]),
        make_float3(lift_model[0], lift_model[1], lift_model[2]),
        make_float2(drag_model[0], drag_model[1]),
        start_vel);

    cuwrap(cudaGetLastError);
    cuwrap(cudaDeviceSynchronize);

    copyGpuCpu(d_angvel, out_angvel.data(), step_count + 1);
    copyGpuCpu(d_aoa, out_aoa.data(), step_count + 1);
    copyGpuCpu(d_acc, out_acc.data(), step_count + 1);
    copyGpuCpu(d_csurf, out_csurf.data(), step_count + 1);
    copyGpuCpu(d_input, out_input.data(), step_count + 1);

    massfree(d_angvel, d_aoa, d_acc, d_csurf, d_input);
        
    // cudaDeviceReset must be called before exiting in order for profiling and
    // tracing tools such as Nsight and Visual Profiler to show complete traces.
    cuwrap(cudaDeviceReset);
}

void raw_execute_cpu(
    float dt,
    int step_count,
    float moi,
    float mass,
    float sas,
    const std::array<float, 3> &rot_model,
    const std::array<float, 3> &lift_model,
    const std::array<float, 2> &drag_model,
    bool aero_model_par,
    float start_vel,
    bool keep_speed,
    float input,
    std::vector<float> &out_angvel,
    std::vector<float> &out_aoa,
    std::vector<float> &out_acc,
    std::vector<float> &out_csurf,
    std::vector<float> &out_input)
{
    //cudaError r;
    h_aero_model = aero_model_par;
    h_spd_const = keep_speed;

    raw_kernel_cpu(
        out_angvel.data(),
        out_aoa.data(),
        out_acc.data(),
        out_csurf.data(),
        out_input.data(),
        dt,
        step_count,
        moi,
        mass,
        sas,
        input,
        make_float3(rot_model[0], rot_model[1], rot_model[2]),
        make_float3(lift_model[0], lift_model[1], lift_model[2]),
        make_float2(drag_model[0], drag_model[1]),
        start_vel);
}