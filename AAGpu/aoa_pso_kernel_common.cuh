#include "cuda_wrappers.cuh"
#include "aircraftmodel.cuh"
#include "aoa_ctrl_constants.h"
#include "ang_vel_pitch.cuh"
#include "aoa_ctrl.cuh"
#include "math_constants.h"
#include "curand.h"
#include "curand_kernel.h"




#ifdef AOAPSOKERNELGPU
#define PREFIX __global__
#define FUNCNAME aoa_pso_kernel
#define FUNCNAME2 aoa_pso_outer_kernel
#define FUNCNAME3 aoa_pso_sort_kernel
#define FUNCNAME4 aoa_pso_update_kernel
#else
#define PREFIX
#define FUNCNAME aoa_pso_kernel_cpu
#define FUNCNAME2 aoa_pso_outer_kernel_cpu
#define FUNCNAME3 aoa_pso_sort_kernel_cpu
#define FUNCNAME4 aoa_pso_update_kernel_cpu
#endif // AOAPSOKERNELGPU

PREFIX void FUNCNAME(
#ifndef AOAPSOKERNELGPU
    int pi,
#endif // !AOAPSOKERNELGPU
    pitch_model *corpus,
    matrix<AOAPARS, 1> *particles,
    float *outputs,
    int model_index,
    float dt,
    int step_count,
    int aoa_divisions,
    float4 weights)
{
#ifdef AOAPSOKERNELGPU
    int pi = blockIdx.x * blockDim.x + threadIdx.x;
#endif
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
    aoa_c.params = particles[pi];

    float result = 0.0f;

    // let's get our AoA bounds
    pitch_model model = corpus[model_index];
    model.preupdate(dt);
    vel_c.preupdatev(&model);
    float min_aoa = vel_c.res_min_aoa;
    float max_aoa = vel_c.res_max_aoa;
    float aoa_step = (max_aoa - min_aoa) / (float)(aoa_divisions - 1);

    // experiment scheme:
    // we have aoa_divisions marks from min_aoa to max_aoa
    // we will perform experiments as transitions from current
    // AoA mark to every other AoA mark.
    // Then we will perform special case experiments (zero AoA stability)

    int exper_count = 0;
    for (int i = 0; i < aoa_divisions; i++)
        for (int j = 0; j < aoa_divisions; j++)
        {
            if (i == j)
                continue;
            float lres = 0.0f;
            float target_aoa = min_aoa + j * aoa_step;
            // local aircraft model
            model = corpus[model_index];
            float start_aoa = min_aoa + i * aoa_step;
            model.pitch_angle = start_aoa;
            // let's set initial control to equilibrium one
            model.preupdate(dt);
            model.csurf_state = aoa_ctrl::get_equlibr(&model, model.pitch_angle)(1, 0);
            // perform simulation
            for (int s = 0; s < step_count; s++)
            {
                model.preupdate(dt);
                float ctl = aoa_c.eval(&model, &vel_c, target_aoa, 0.0f, dt);
                model.simulation_step(dt, ctl);
                float err = (model.aoa - target_aoa);
                float diff = err * err;
                if ((model.aoa - target_aoa) * (start_aoa - target_aoa) < 0.0f)
                    diff *= weights.w;
                lres += diff * dt * s;
            }
            exper_count++;
            result += lres;
        }

    // special case - zero start aoa, zero target aoa
    float target_aoa = 0.0f;
    model = corpus[model_index];
    model.pitch_angle = 0.0f;
    model.preupdate(dt);
    model.csurf_state = aoa_ctrl::get_equlibr(&model, model.pitch_angle)(1, 0);
    float lres = 0.0f;
    for (int s = 0; s < step_count; s++)
    {
        model.preupdate(dt);
        float ctl = aoa_c.eval(&model, &vel_c, target_aoa, 0.0f, dt);
        model.simulation_step(dt, ctl);
        float err = (model.aoa - target_aoa);
        float diff = err * err;
        lres += diff * dt * s;
    }
    lres *= weights.x;
    result += lres;
    exper_count++;

    // scale by experiment count
    result = result / (float)exper_count;

    outputs[pi] += result;
}


PREFIX void FUNCNAME2(
#ifndef AOAPSOKERNELGPU
    int pi,
#endif // !AOAPSOKERNELGPU
    matrix<AOAPARS, 1> *particles,
    matrix<AOAPARS, 1> *best_particles,
    float *outputs,
    float *best_outputs)
{
#ifdef AOAPSOKERNELGPU
    int pi = blockIdx.x * blockDim.x + threadIdx.x;
#endif
    if (outputs[pi] < best_outputs[pi])
    {
        best_outputs[pi] = outputs[pi];
        best_particles[pi] = particles[pi];
    }
}

PREFIX void FUNCNAME3(
    float *best_outputs,
    int *best_particle,
    int particle_count)
{
    int global_best_index = 0;
    float best_value = CUDART_INF_F;
    for (int i = 0; i < particle_count; i++)
        if (best_outputs[i] < best_value)
        {
            global_best_index = i;
            best_value = best_outputs[i];
        }
    *best_particle = global_best_index;
}

PREFIX void FUNCNAME4(
#ifndef AOAPSOKERNELGPU
    int pi,
#endif // !AOAPSOKERNELGPU
    matrix<AOAPARS, 1> *particles,
    matrix<AOAPARS, 1> *velocities,
    matrix<AOAPARS, 1> *best_particles,
    float *outputs,
    float *best_outputs,
    int *best_particle,
    float w,
    float c1,
    float c2,
    unsigned long long seed,
    unsigned long long offset)
{
#ifdef AOAPSOKERNELGPU
    int pi = blockIdx.x * blockDim.x + threadIdx.x;
#endif
    // generate random vectors b1 and b2
    curandStateXORWOW_t rand_state;
    curand_init(seed, pi, offset, &rand_state);
    /*matrix<AOAPARS, 1> b1, b2;
    for (int i = 0; i < AOAPARS; i++)
    {
        b1(i, 0) = 2.0f * (curand_uniform(&rand_state) - 0.5f);
        b2(i, 0) = 2.0f * (curand_uniform(&rand_state) - 0.5f);
    }*/
    float b1 = curand_uniform(&rand_state);
    float b2 = curand_uniform(&rand_state);

    // update velocities
    matrix<AOAPARS, 1> local_best = best_particles[pi];
    matrix<AOAPARS, 1> global_best = best_particles[*best_particle];
    matrix<AOAPARS, 1> vel = velocities[pi];
    vel = vel * w + (local_best - particles[pi]) * c1 * b1 +
        (global_best - particles[pi]) * c2 * b2;
    // update particles
    particles[pi] = particles[pi] + vel;
    velocities[pi] = vel;
    outputs[pi] = 0.0f;
}