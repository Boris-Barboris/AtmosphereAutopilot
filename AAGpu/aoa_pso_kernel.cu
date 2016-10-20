#include "AAGpu.h"

#define AOAPSOKERNELGPU
#include "aoa_pso_kernel_common.cuh"

#include <thread>
#include <ctime>


static std::thread *aoa_pso_thread = new std::thread();
static bool stop_flag = false;


void do_start_aoa_pso(
    float dt,
    int step_count,
    float moi,
    float mass,
    float sas,
    std::array<float, 3> rot_model,
    std::array<float, 3> lift_model,
    std::array<float, 2> drag_model,
    bool a_model,
    float start_vel,
    bool keep_speed,
    std::array<std::tuple<float, float>, AOAINPUTS> input_norms,
    std::array<std::tuple<float, float>, AOAOUTPUTS> output_norms,
    int prtcl_blocks,
    float w,
    float c1,
    float c2,
    report_dlg repotrer)
{
    // initialize norms
    matrix<AOAINPUTS, 2> in_norms;
    for (int i = 0; i < AOAINPUTS; i++)
    {
        in_norms(i, 0) = std::get<0>(input_norms[i]);
        in_norms(i, 1) = std::get<1>(input_norms[i]);
    }
    matrix<AOAOUTPUTS, 2> out_norms;
    for (int i = 0; i < AOAOUTPUTS; i++)
    {
        out_norms(i, 0) = std::get<0>(output_norms[i]);
        out_norms(i, 1) = std::get<1>(output_norms[i]);
    }

    // initialize models
    pitch_model *models;
    massalloc_cpu(1, &models);
    models[0].zero_init();
    models[0].velocity.x = start_vel;
    models[0].moi = moi;
    models[0].rot_m = make_float3(rot_model[0], rot_model[1], rot_model[2]);
    models[0].lift_m = make_float3(lift_model[0], lift_model[1], lift_model[2]);
    models[0].drag_m = make_float2(drag_model[0], drag_model[1]);
    models[0].sas_torque = sas;
    models[0].mass = mass;

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
    curandGenerator_t generator;
    curandCreateGenerator(&generator, curandRngType::CURAND_RNG_PSEUDO_XORWOW);
    curandSetPseudoRandomGeneratorSeed(generator, seed);
    curandGenerateUniform(generator, (float*)particles, 
        prtcl_blocks * PARTICLEBLOCK * AOAPARS);
    curandGenerateUniform(generator, (float*)velocities,
        prtcl_blocks * PARTICLEBLOCK * AOAPARS);
    for (int i = 0; i < prtcl_blocks * PARTICLEBLOCK * AOAPARS; i++)
    {
        ((float*)particles)[i] = 2.0f * (((float*)particles)[i] - 0.5f);
        ((float*)best_particles)[i] = ((float*)particles)[i];
        ((float*)velocities)[i] = 0.05f * ((float*)velocities)[i];  // smaller velocities
    }

    // allocate GPU memory
    cuwrap(cudaSetDevice, 0);

    pitch_model *corpus;
    matrix<AOAPARS, 1> *d_particles, *d_best_particles, *d_velocities;
    float *d_outputs, *d_best_outputs;
    int *d_best_index;

    massalloc(1, &corpus);
    massalloc(prtcl_blocks * PARTICLEBLOCK, &d_particles, &d_best_particles, 
        &d_velocities, &d_outputs, &d_best_outputs);
    massalloc(1, &d_best_index);

    // initialize it
    copyCpuGpu(models, corpus, 1);
    copyCpuGpu(particles, d_particles, prtcl_blocks * PARTICLEBLOCK);
    copyCpuGpu(best_particles, d_best_particles, prtcl_blocks * PARTICLEBLOCK);
    copyCpuGpu(velocities, d_velocities, prtcl_blocks * PARTICLEBLOCK);
    copyCpuGpu(outputs, d_outputs, prtcl_blocks * PARTICLEBLOCK);
    copyCpuGpu(best_outputs, d_best_outputs, prtcl_blocks * PARTICLEBLOCK);

    cudaMemcpyToSymbol(d_aero_model, &a_model, sizeof(bool));
    cudaMemcpyToSymbol(d_spd_const, &keep_speed, sizeof(bool));

    // main cycle
    unsigned long long offset = prtcl_blocks * PARTICLEBLOCK * AOAPARS;
    int epoch = 0;
    while (!stop_flag)
    {
        aoa_pso_kernel<<<prtcl_blocks, PARTICLEBLOCK>>>(
            corpus,
            d_particles,
            in_norms,
            out_norms,
            d_outputs,
            0,
            dt,
            step_count,
            make_float4(10.0f, 1.0f, 0.6f, 1000.0f));

        aoa_pso_outer_kernel<<<prtcl_blocks, PARTICLEBLOCK>>>(
            d_particles,
            d_best_particles,
            d_outputs,
            d_best_outputs);

        aoa_pso_sort_kernel<<<1, 1>>>(
            d_best_particles,
            d_best_outputs,
            d_best_index,
            prtcl_blocks * PARTICLEBLOCK);

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

        offset += prtcl_blocks * PARTICLEBLOCK * AOAPARS;

        cuwrap(cudaGetLastError);
        cuwrap(cudaDeviceSynchronize);
        // get best point
        int best_index = 0;
        copyGpuCpu(d_best_index, &best_index, 1);
        matrix<AOAPARS, 1> best_particle;
        copyGpuCpu(d_best_particles + best_index * sizeof(matrix<AOAPARS, 1>), 
            &best_particle, 1);
        float best_target_func = 0.0f;
        copyGpuCpu(d_best_outputs + best_index * sizeof(float),
            &best_target_func, 1);
        // report to caller
        std::array<float, AOAPARS> best_particle_arr;
        for (int i = 0; i < AOAPARS; i++)
            best_particle_arr[i] = best_particle(i, 0);
        repotrer(epoch++, best_target_func, best_particle_arr);
    }

    // Clean up
    massfree(corpus, d_particles, d_best_particles, d_velocities, d_outputs, 
        d_best_outputs, d_best_index);
    massfree_cpu(models, particles, best_particles, velocities, outputs,
        best_outputs);
    cuwrap(cudaDeviceReset);
}



void start_aoa_pso(
    float dt,
    int step_count,
    float moi,
    float mass,
    float sas,
    const std::array<float, 3> &rot_model,
    const std::array<float, 3> &lift_model,
    const std::array<float, 2> &drag_model,
    bool a_model,
    float start_vel,
    bool keep_speed,
    const std::array<std::tuple<float, float>, AOAINPUTS> &input_norms,
    const std::array<std::tuple<float, float>, AOAOUTPUTS> &output_norms,
    int prtcl_blocks,
    float w,
    float c1,
    float c2,
    report_dlg repotrer)
{
    if (aoa_pso_thread->joinable())
        return;
    delete aoa_pso_thread;
    stop_flag = false;
    aoa_pso_thread = new std::thread(do_start_aoa_pso,
        dt, step_count, moi, mass, sas, rot_model, lift_model, drag_model,
        a_model, start_vel, keep_speed, input_norms, output_norms,
        prtcl_blocks, w, c1, c2, repotrer);
}

void stop_aoa_pso()
{
    if (aoa_pso_thread->joinable())
    {
        stop_flag = true;
        aoa_pso_thread->join();
    }
}