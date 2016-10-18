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
    matrix<AOAPARS, 1> aoa_pars,
    matrix<AOAINPUTS, 2> input_norms,
    matrix<AOAOUTPUTS, 2> output_norms,
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
    //aoa_c.params = aoa_pars;
    aoa_c.net.input_norm = input_norms;
    aoa_c.net.output_norm = output_norms;
    aoa_c.net.init(aoa_pars);

    // simulate
    angvel_output[0] = model.ang_vel;
    aoa_output[0] = model.aoa;
    acc_output[0] = model.ang_acc;
    csurf_output[0] = model.csurf_state;
    input_output[0] = 0.0f;
    output_vel_output[0] = 0.0f;
    for (int i = 0; i < step_count; i++)
    {
        model.preupdate(dt);
        float ctl = aoa_c.eval(&model, &vel_c, input, 0.0f, dt);
        //ctl = ang_acc_p::eval_ac(model, input, dt);
        //model.simulation_step(dt, ctl);
        model.simulation_step(dt, ctl);
        angvel_output[i + 1] = model.ang_vel;
        aoa_output[i + 1] = model.aoa;
        acc_output[i + 1] = model.ang_acc;
        csurf_output[i + 1] = model.csurf_state;
        input_output[i + 1] = ctl;
        output_vel_output[i + 1] = aoa_c.output_vel;
    }
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
    const std::array<float, AOAPARS> &aoa_params,
    const std::array<std::tuple<float, float>, AOAINPUTS> &input_norms,
    const std::array<std::tuple<float, float>, AOAOUTPUTS> &output_norms,
    std::vector<float> &out_angvel,
    std::vector<float> &out_aoa,
    std::vector<float> &out_acc,
    std::vector<float> &out_csurf,
    std::vector<float> &out_input,
    std::vector<float> &out_vel_output)
{
    matrix<AOAPARS, 1> aoa_params_mat;
    for (int i = 0; i < AOAPARS; i++)
        aoa_params_mat(i, 0) = aoa_params[i];
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

#ifdef AOAKERNELGPU
    float *d_angvel, *d_aoa, *d_acc, *d_csurf, *d_input, *d_out_vel;

    cuwrap(cudaSetDevice, 0);
    massalloc(step_count + 1, &d_angvel, &d_aoa, &d_acc, &d_csurf, 
        &d_input, &d_out_vel);

    //cudaError r;
    cudaMemcpyToSymbol(d_aero_model, &aero_model_par, sizeof(bool));
    cudaMemcpyToSymbol(d_spd_const, &keep_speed, sizeof(bool));    

    aoa_eval_kernel <<<1, 1>>>(
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
        in_norms,
        out_norms,
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
        in_norms,
        out_norms,
        start_vel,
        start_aoa);
#endif // AOAKERNELGPU
}