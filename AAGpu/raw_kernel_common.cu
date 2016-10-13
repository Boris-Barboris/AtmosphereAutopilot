RAWPREFIX void RAWFUNCNAME(
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
    RAWCONTEXTPREFIX thread_context context;
    pitch_model& model = context.mdl;
    model.zero_init();

    ang_vel_p &vel_c = context.vel_c;
    vel_c.zero_init();


    // initialize model    
    model.velocity.x = start_vel;
    model.moi = moi;
    model.rot_m = rot_m;
    model.lift_m = lift_m;
    model.drag_m = drag_m;
    model.sas_torque = sas;
    model.mass = mass;

    // and vel_c
    vel_c.max_v_construction = 0.5f;
    vel_c.max_aoa = 0.25f;
    vel_c.quadr_Kp = 0.45f;
    vel_c.moderate_aoa = true;

    // simulate
    angvel_output[0] = model.ang_vel;
    aoa_output[0] = model.aoa;
    acc_output[0] = model.ang_acc;
    csurf_output[0] = model.csurf_state;
    input_output[0] = 0.0f;
    for (int i = 0; i < step_count; i++)
    {
        model.preupdate(dt);
        float ctl = vel_c.eval(&model, input, 0.0f, dt);
        //ctl = ang_acc_p::eval_ac(model, input, dt);
        model.simulation_step(dt, ctl);
        //model.simulation_step(dt, input);
        angvel_output[i + 1] = model.ang_vel;
        aoa_output[i + 1] = model.aoa;
        acc_output[i + 1] = model.ang_acc;
        csurf_output[i + 1] = model.csurf_state;
        input_output[i + 1] = ctl;
    }
}