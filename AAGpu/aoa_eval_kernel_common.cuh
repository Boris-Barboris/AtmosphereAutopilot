#include "cuda_wrappers.cuh"
#include "aircraftmodel.cuh"

#ifdef AOAKERNELGPU
#define RAWPREFIX __global__
#define RAWFUNCNAME aoa_eval_kernel
#define RAWEXECFUNCNAME aoa_execute
#define RAWCONTEXTPREFIX __shared__
#else
#define RAWPREFIX 
#define RAWFUNCNAME aoa_eval_kernel_cpu
#define RAWEXECFUNCNAME aoa_execute_cpu
#define RAWCONTEXTPREFIX 
#endif // AOAKERNELGPU


RAWPREFIX void RAWFUNCNAME(
    float *angvel_output,
    float *aoa_output,
    float *acc_output,
    float *csurf_output,
    float *input_output,
    float *output_vel_output,
    float dt,
    int step_count,
    float moi,
    float mass,
    float sas,
    float input,
    float3 rot_m,
    float3 lift_m,
    float2 drag_m,
    matrix<AOALINPARAMS, 1> aoa_pars,
    float start_vel,
    float start_aoa)
{
    RAWCONTEXTPREFIX thread_context context;
    pitch_model& model = context.mdl;
    model.zero_init();

    ang_vel_p &vel_c = context.vel_c;
    vel_c.zero_init();

    aoa_ctrl &aoa_c = context.aoa_c;
    aoa_c.zero_init();


    // initialize model    
    model.velocity.x = start_vel;
    model.moi = moi;
    model.rot_m = rot_m;
    model.lift_m = lift_m;
    model.drag_m = drag_m;
    model.sas_torque = sas;
    model.mass = mass;
    model.pitch_angle = start_aoa;
    model.aoa = start_aoa;

    // vel_c
    vel_c.max_v_construction = 0.5f;
    vel_c.max_aoa = 0.25f;
    vel_c.quadr_Kp = 0.45f;
    vel_c.moderate_aoa = true;

    // aoa_c
    aoa_c.params = aoa_pars;

    // simulate
    input_output[0] = 0.0f;
    for (int i = 0; i < step_count; i++)
    {
        model.preupdate(dt);
        angvel_output[i] = model.ang_vel;
        aoa_output[i] = model.aoa;
        acc_output[i] = model.ang_acc;
        csurf_output[i] = model.csurf_state;
        float ctl = aoa_c.eval(&model, &vel_c, input, 0.0f, dt);
        model.simulation_step(dt, ctl);
        output_vel_output[i + 1] = aoa_c.output_vel;
        input_output[i + 1] = ctl;
    }
    model.preupdate(dt);
    angvel_output[step_count] = model.ang_vel;
    aoa_output[step_count] = model.aoa;
    acc_output[step_count] = model.ang_acc;
    csurf_output[step_count] = model.csurf_state;
}



void RAWEXECFUNCNAME(
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
    float start_aoa,
    bool keep_speed,
    float input,
    const std::array<float, AOALINPARAMS> &aoa_params,
    std::vector<float> &out_angvel,
    std::vector<float> &out_aoa,
    std::vector<float> &out_acc,
    std::vector<float> &out_csurf,
    std::vector<float> &out_input,
    std::vector<float> &out_vel_output)
{
    matrix<AOALINPARAMS, 1> aoa_params_mat;
    for (int i = 0; i < AOALINPARAMS; i++)
        aoa_params_mat(i, 0) = aoa_params[i];

#ifdef AOAKERNELGPU
    float *d_angvel, *d_aoa, *d_acc, *d_csurf, *d_input, *d_out_vel;

    cuwrap(cudaSetDevice, 0);
    massalloc(step_count + 1, &d_angvel, &d_aoa, &d_acc, &d_csurf, 
        &d_input, &d_out_vel);

    //cudaError r;
    cudaMemcpyToSymbol(d_aero_model, &aero_model_par, sizeof(bool));
    cudaMemcpyToSymbol(d_spd_const, &keep_speed, sizeof(bool));

    aoa_eval_kernel<<<1, 1>>>(
        d_angvel,
        d_aoa,
        d_acc,
        d_csurf,
        d_input,
        d_out_vel,
        dt,
        step_count,
        moi,
        mass,
        sas,
        input,
        make_float3(rot_model[0], rot_model[1], rot_model[2]),
        make_float3(lift_model[0], lift_model[1], lift_model[2]),
        make_float2(drag_model[0], drag_model[1]),
        aoa_params_mat,
        start_vel,
        start_aoa);

    cuwrap(cudaGetLastError);
    cuwrap(cudaDeviceSynchronize);

    copyGpuCpu(d_angvel, out_angvel.data(), step_count + 1);
    copyGpuCpu(d_aoa, out_aoa.data(), step_count + 1);
    copyGpuCpu(d_acc, out_acc.data(), step_count + 1);
    copyGpuCpu(d_csurf, out_csurf.data(), step_count + 1);
    copyGpuCpu(d_input, out_input.data(), step_count + 1);
    copyGpuCpu(d_out_vel, out_vel_output.data(), step_count + 1);

    massfree(d_angvel, d_aoa, d_acc, d_csurf, d_input, d_out_vel);

    // cudaDeviceReset must be called before exiting in order for profiling and
    // tracing tools such as Nsight and Visual Profiler to show complete traces.
    cuwrap(cudaDeviceReset);
#else
    h_aero_model = aero_model_par;
    h_spd_const = keep_speed;

    aoa_eval_kernel_cpu(
        out_angvel.data(),
        out_aoa.data(),
        out_acc.data(),
        out_csurf.data(),
        out_input.data(),
        out_vel_output.data(),
        dt,
        step_count,
        moi,
        mass,
        sas,
        input,
        make_float3(rot_model[0], rot_model[1], rot_model[2]),
        make_float3(lift_model[0], lift_model[1], lift_model[2]),
        make_float2(drag_model[0], drag_model[1]),
        aoa_params_mat,
        start_vel,
        start_aoa);
#endif // AOAKERNELGPU
}