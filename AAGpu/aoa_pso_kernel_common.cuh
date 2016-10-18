#include "cuda_wrappers.cuh"
#include "aircraftmodel.cuh"
#include "aoa_ctrl_constants.h"
#include "ang_vel_pitch.cuh"
#include "aoa_ctrl.cuh"



#ifdef AOAPSOKERNELGPU
#define PREFIX __global__
#define FUNCNAME aoa_pso_kernel
#else
#define PREFIX
#define FUNCNAME aoa_pso_kernel_cpu
#endif // AOAPSOKERNELGPU

PREFIX void FUNCNAME(
#ifndef AOAPSOKERNELGPU
    int pi,
#endif
    pitch_model *corpus,
    matrix<AOAPARS, 1> *particles,
    matrix<AOAINPUTS, 2> input_norms,
    matrix<AOAOUTPUTS, 2> output_norms,
    float *outputs,
    int model_index,
    float dt,
    int step_count,
    float4 weights)
{
#ifdef AOAPSOKERNELGPU
    int pi = blockIdx.x * blockDim.x + threadIdx.x;
#endif
    // local aircraft model
    pitch_model model = corpus[model_index];
    // local controllers
    ang_vel_p vel_c;
    vel_c.zero_init();
    aoa_ctrl aoa_c;
    aoa_c.zero_init();

    //vel_c
    vel_c.max_v_construction = 0.5f;
    vel_c.max_aoa = 0.25f;
    vel_c.quadr_Kp = 0.45f;
    vel_c.moderate_aoa = true;

    // aoa_c
    aoa_c.net.input_norm = input_norms;
    aoa_c.net.output_norm = output_norms;
    aoa_c.net.init(particles[pi]);

    // First pass, zero AoA maintance
    float func_result1 = 0.0f;
    for (int i = 0; i < step_count; i++)
    {
        model.preupdate(dt);
        float ctl = aoa_c.eval(&model, &vel_c, 0.0f, 0.0f, dt);
        model.simulation_step(dt, ctl);
        float diff = model.aoa * model.aoa;
        if (model.aoa < 0.0f)
            diff *= weights.w;
        func_result1 += diff;
    }
    func_result1 *= weights.x;

    // Second pass, zero AoA to max AoA
    float func_result2 = 0.0f;
    model = corpus[model_index];
    vel_c.preupdatev(&model);
    float target_aoa = vel_c.res_max_aoa;
    for (int i = 0; i < step_count; i++)
    {
        model.preupdate(dt);
        float ctl = aoa_c.eval(&model, &vel_c, 0.0f, 0.0f, dt);
        model.simulation_step(dt, ctl);
        float diff = model.aoa * model.aoa;
        if (model.aoa > target_aoa)
            diff *= weights.w;
        func_result2 += diff;
    }
    func_result2 *= weights.y;
}