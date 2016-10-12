#include "AAGpu.h"

#include "cuda_wrappers.hpp"
#include "aircraftmodel.cuh"


__global__ void raw_kernel(
    float *angvel_output, 
    float *aoa_output, 
    float *acc_output, 
    float *csurf_output, 
    float *input_output, 
    float dt, 
    int step_count, 
    float moi, 
    float mass, 
    float sas, 
    float input, 
    float3 rot_m, 
    float3 lift_m, 
    float2 drag_m, 
    float start_vel)
{
    __shared__ pitch_model model;
    // initialize model
    model.zero_init();
    model.velocity.x = start_vel;
    model.moi = moi;
    model.rot_m = rot_m;
    model.lift_m = lift_m;
    model.drag_m = drag_m;
    model.sas_torque = sas;
    model.mass = mass;

    // simulate
    angvel_output[0] = model.ang_vel;
    aoa_output[0] = model.aoa;
    acc_output[0] = model.ang_acc;
    csurf_output[0] = model.csurf_state;
    input_output[0] = 0.0f;
    for (int i = 0; i < step_count; i++)
    {
        model.preupdate(dt);
        model.simulation_step(dt, input);
        angvel_output[i + 1] = model.ang_vel;
        aoa_output[i + 1] = model.aoa;
        acc_output[i + 1] = model.ang_acc;
        csurf_output[i + 1] = model.csurf_state;
        input_output[i + 1] = input;
    }
}


void raw_execute(
    float dt,
    int step_count,
    float moi,
    float mass,
    float sas,
    const std::array<float, 3> &rot_model,
    const std::array<float, 3> &lift_model,
    const std::array<float, 2> &drag_model,
    bool aero_model,
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

    cudaError r;
    r = cudaMemcpyToSymbol(::aero_model, &aero_model, sizeof(bool));
    r = cudaMemcpyToSymbol(::spd_const, &keep_speed, sizeof(bool));

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