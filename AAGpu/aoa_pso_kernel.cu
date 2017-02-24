#include "AAGpu.h"

#define AOAPSOKERNELGPU
#include "aoa_pso_kernel_common.cuh"

#include <thread>
#include <ctime>
#include <random>


static std::thread *aoa_pso_thread = new std::thread();
static bool stop_flag = false;


float4 make_float4(const std::array<float, 4> &arr)
{
    return make_float4(arr[0], arr[1], arr[2], arr[3]);
}

void do_start_aoa_pso(
    float dt,
    int step_count,
    const pitch_model_params &corpus,
    bool a_model,
    float start_vel,
    bool keep_speed,
    int prtcl_blocks,
    float w,
    float c1,
    float c2,
    float initial_span,
    int aoa_divisions,
    std::array<float, 4> exper_weights,
    report_dlg repotrer,
    int iter_limit)
{
    // initialize models
    pitch_model *models;
    int model_count = 1;
    massalloc_cpu(model_count, &models);
    for (int i = 0; i < model_count; i++)
    {
        models[i].zero_init();
        models[i].velocity.x = start_vel;
        models[i].moi = corpus.moi;
        models[i].rot_m = make_float3(corpus.rot_model);
        models[i].lift_m = make_float3(corpus.lift_model);
        models[i].drag_m = make_float2(corpus.drag_model);
        models[i].sas_torque = corpus.sas;
        models[i].mass = corpus.mass;
    }

    // initialize particles
    matrix<AOAPARS, 1> *particles, *best_particles, *velocities;
    float *outputs, *best_outputs;
    massalloc_cpu(prtcl_blocks * PARTICLEBLOCK, &particles, &best_particles, 
        &velocities, &outputs, &best_outputs);
    for (int i = 0; i < prtcl_blocks * PARTICLEBLOCK; i++)
    {
        outputs[i] = 0.0f;
        best_outputs[i] = std::numeric_limits<float>::infinity();
    }
    
    // randomize
    unsigned long long seed = (long long)std::time(nullptr);
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> rng(-1.0f, 1.0f);
    for (int i = 0; i < prtcl_blocks * PARTICLEBLOCK; i++)
        for (int j = 0; j < AOAPARS; j++)
        {
            particles[i](j, 0) = initial_span * rng(gen);
            best_particles[i](j, 0) = particles[i](j, 0);
            velocities[i](j, 0) = 0.2f * initial_span * rng(gen);
        }        

    // allocate GPU memory
    cuwrap(cudaSetDevice, 0);

    pitch_model *d_corpus;
    matrix<AOAPARS, 1> *d_particles, *d_best_particles, *d_velocities;
    float *d_outputs, *d_best_outputs;
    int *d_best_index;

    massalloc(model_count, &d_corpus);
    massalloc(prtcl_blocks * PARTICLEBLOCK, &d_particles, &d_best_particles, 
        &d_velocities, &d_outputs, &d_best_outputs);
    massalloc(1, &d_best_index);

    // initialize it
    copyCpuGpu(models, d_corpus, model_count);
    copyCpuGpu(particles, d_particles, prtcl_blocks * PARTICLEBLOCK);
    copyCpuGpu(best_particles, d_best_particles, prtcl_blocks * PARTICLEBLOCK);
    copyCpuGpu(velocities, d_velocities, prtcl_blocks * PARTICLEBLOCK);
    copyCpuGpu(outputs, d_outputs, prtcl_blocks * PARTICLEBLOCK);
    copyCpuGpu(best_outputs, d_best_outputs, prtcl_blocks * PARTICLEBLOCK);

    cudaMemcpyToSymbol(d_aero_model, &a_model, sizeof(bool));
    cudaMemcpyToSymbol(d_spd_const, &keep_speed, sizeof(bool));

    // main cycle
    unsigned long long offset = prtcl_blocks * PARTICLEBLOCK;
    int epoch = 0;
    int cycles_in_vain = 0;
    float global_best = std::numeric_limits<float>::infinity();
    while (!stop_flag && epoch < iter_limit)
    {
        int m_index = 0;
        while (m_index < model_count)
        {
            if (stop_flag)
                goto cleanup_label;
            // lauch kernel
            aoa_pso_kernel<<<prtcl_blocks, PARTICLEBLOCK>>> (
                d_corpus,
                d_particles,
                d_outputs,
                m_index,
                dt,
                step_count,
                aoa_divisions,
                make_float4(exper_weights));
            cuwrap(cudaGetLastError);
            cuwrap(cudaDeviceSynchronize);
            m_index++;
        }

        aoa_pso_outer_kernel<<<prtcl_blocks, PARTICLEBLOCK>>>(
            d_particles,
            d_best_particles,
            d_outputs,
            d_best_outputs);

        cuwrap(cudaGetLastError);

        aoa_pso_sort_kernel<<<1, 1>>>(
            d_best_outputs,
            d_best_index,
            prtcl_blocks * PARTICLEBLOCK);

        cuwrap(cudaGetLastError);

        aoa_pso_update_kernel<<<prtcl_blocks, PARTICLEBLOCK>>>(
            d_particles,
            d_velocities,
            d_best_particles,
            d_outputs,
            d_best_outputs,
            d_best_index,
            w,
            c1,
            c2,
            seed,
            offset);

        offset += prtcl_blocks * PARTICLEBLOCK * 2;

        cuwrap(cudaGetLastError);
        cuwrap(cudaDeviceSynchronize);

        int best_index = 0;
        copyGpuCpu(d_best_index, &best_index, 1);
        matrix<AOAPARS, 1> best_particle;
        copyGpuCpu(d_best_particles + best_index, &best_particle, 1);
        float best_target_func = 0.0f;
        copyGpuCpu(d_best_outputs + best_index, &best_target_func, 1);
        
        // report to caller
        std::array<float, AOAPARS> best_particle_arr;
        for (int i = 0; i < AOAPARS; i++)
            best_particle_arr[i] = best_particle(i, 0);
        repotrer(epoch++, best_target_func, best_particle_arr);

        // check if we didn't improve
        if (best_target_func < global_best)
        {
            global_best = best_target_func;
            cycles_in_vain = 0;
        }
        else
            cycles_in_vain++;
        // rerandomize particles if needed
        if (cycles_in_vain >= 100)
        {
            cycles_in_vain = 0;
            for (int i = 0; i < prtcl_blocks * PARTICLEBLOCK; i++)
                for (int j = 0; j < AOAPARS; j++)
                {
                    particles[i](j, 0) = initial_span * rng(gen) + best_particle(j, 0);
                    velocities[i](j, 0) = 0.2f * initial_span * rng(gen);
                }
            copyCpuGpu(particles, d_particles, prtcl_blocks * PARTICLEBLOCK);
            copyCpuGpu(velocities, d_velocities, prtcl_blocks * PARTICLEBLOCK);
        }
    }

    cleanup_label:

    // Clean up
    massfree(d_corpus, d_particles, d_best_particles, d_velocities, d_outputs,
        d_best_outputs, d_best_index);
    massfree_cpu(models, particles, best_particles, velocities, outputs,
        best_outputs);
    cuwrap(cudaDeviceReset);
}



bool start_aoa_pso(
    float dt,
    int step_count,
    const pitch_model_params &model_params,
    bool a_model,
    float start_vel,
    bool keep_speed,
    int prtcl_blocks,
    float w,
    float c1,
    float c2,
    float initial_span,
    int aoa_divisions,
    const std::array<float, 4> &exper_weights,
    report_dlg repotrer,
    int iter_limit)
{
    if (aoa_pso_thread->joinable())
        return false;
    delete aoa_pso_thread;
    stop_flag = false;
    aoa_pso_thread = new std::thread(do_start_aoa_pso,
        dt, step_count, model_params, a_model, start_vel, keep_speed, 
        prtcl_blocks, w, c1, c2, initial_span, aoa_divisions, exper_weights, repotrer,
        iter_limit);
    return true;
}

void stop_aoa_pso()
{
    if (aoa_pso_thread->joinable())
    {
        stop_flag = true;
        aoa_pso_thread->join();
    }
}