#include "aoa_ctrl.cuh"

__device__ __host__ void aoa_ctrl::update_pars(pitch_model *mdl)
{
    if (already_preupdated)
    {
        already_preupdated = false;
        return;
    }
    cur_aoa_equilibr = aoa_ctrl::get_equlibr(mdl, mdl->aoa)(0, 0);
}

__device__ __host__ matrix<2, 1> aoa_ctrl::get_equlibr(pitch_model *mdl, float aoa)
{
    matrix<2, 2> eq_A;
    matrix<2, 1> eq_B;
    eq_A(0, 0) = mdl->A(0, 1);
    eq_A(0, 1) = mdl->A(0, 2) + mdl->B(0, 0);
    eq_A(1, 0) = mdl->A(1, 1);
    eq_A(1, 1) = mdl->A(1, 2) + mdl->B(1, 0);
    eq_B(0, 0) = -(mdl->A(0, 0) * aoa + mdl->C(0, 0));
    eq_B(1, 0) = -(mdl->A(1, 0) * aoa + mdl->C(1, 0));
    matrix<2, 1> eq_x = eq_A / eq_B;
    return eq_x;
}

__device__ __host__ static float get_equlibr_vel(pitch_model *mdl, float aoa, float ctl)
{
    return -(mdl->A(0, 0) * aoa + mdl->A(0, 2) * ctl + mdl->C(0, 0));
}

__device__ __host__ static float predict_aoa(pitch_model *mdl, float ctrl, float dt)
{
    float csurf;
    if (aero_model)
        csurf = mdl->csurf_state;
    else
        csurf = moveto(mdl->csurf_state, ctrl, dt * stock_csurf_spd);
    float pred_aoa = (float)(mdl->A.rowSlice<0>() * colVec(mdl->aoa, mdl->ang_vel, csurf)) +
        mdl->B(0, 0) * ctrl + mdl->C(0, 0);
    return mdl->aoa + pred_aoa * dt;
}

# define AOAPCITER 2

__device__ __host__ float aoa_ctrl::eval(pitch_model *mdl, ang_vel_p *vel_c, 
    float target, float target_deriv, float dt)
{
    vel_c->preupdatev(mdl);
    update_pars(mdl);    
    target_aoa = clamp(target, vel_c->res_min_aoa, vel_c->res_max_aoa);    

    float cur_aoa = mdl->aoa;
    //float prev_out_vel = output_vel;
    float aoa_err = target_aoa - cur_aoa;

    auto eq_x = get_equlibr(mdl, target);
    float des_aoa_equil = eq_x(0, 0);   // equlibrium angular velocity on target aoa
    float des_aoa_ctl = eq_x(1, 0);     // equilibrium control input
    float abs_err = fabsf(aoa_err);
    // replace ANN approximator with polynom

    float err_sign = copysignf(1.0f, aoa_err);
    float f1_abs_err = powf(abs_err, 2.0f / 3.0f);
    float f3_ctl_err = fabs(des_aoa_ctl - copysignf(1.0f, -aoa_err));
    float f4_vel_err = mdl->ang_vel - des_aoa_equil;
    if (f4_vel_err * aoa_err < 0.0f)
        f4_vel_err = 0.0f;
    //float f3 = 1.0f + fabsf(params(2, 0));
    float abs_output_shift = f1_abs_err * (params(0, 0) + params(1, 0) * abs_err +
        params(2, 0) * f3_ctl_err + params(3, 0) * f4_vel_err);
    float output_shift = err_sign * abs_output_shift;

    //float des_delta_aoa = aoa_err;
    if (abs_output_shift * dt >= 0.9f * abs_err)
    {
        predicted_output = 0.0f;
        output_acc = (predicted_output - output_shift) / dt;
    }
    else
    {

        //// handle discrete time overshoot
        ////if ((aoa_err - f3 * output_shift * dt) * aoa_err < 0.0f)
        ////    output_shift = 1.0f / f3 * aoa_err / dt;

        ////float output_shift = get_output(vel_c, cur_aoa, target, dt);    
        ////float des_aoa_equil = get_equlibr_vel(mdl, target, mdl->csurf_state);

        float shift_ang_vel = mdl->ang_vel - cur_aoa_equilibr;
        predicted_aoa = cur_aoa + shift_ang_vel * dt;
        output_vel = output_shift + des_aoa_equil;

        //for (int i = 0; i < AOAPCITER; i++)
        //{
        //    //if (aoa_err * (target_aoa - predicted_aoa) < 0.0f)
        //    //    predicted_aoa = target_aoa;
        //    //predicted_output = get_output(vel_c, predicted_aoa, target_aoa, dt);
        float pred_error = target_aoa - predicted_aoa;
        float abs_pred_error = fabsf(pred_error);
        f1_abs_err = powf(abs_pred_error, 2.0f / 3.0f);
        f3_ctl_err = fabs(des_aoa_ctl - copysignf(1.0f, -pred_error));
        f4_vel_err = output_vel - des_aoa_equil;
        if (f4_vel_err * pred_error < 0.0f)
            f4_vel_err = 0.0f;
        float pred_err_sign = copysignf(1.0f, pred_error);

        float abs_pred_output = f1_abs_err * (params(0, 0) + params(1, 0) * abs_pred_error +
            params(2, 0) * f3_ctl_err + params(3, 0) * f4_vel_err);
        predicted_output = pred_err_sign * abs_pred_output;

        /*if (abs_pred_output * dt >= 0.5f * abs_pred_error)
            predicted_output = 0.5f * pred_error / dt;*/

        //    //if ((pred_error - f3 * predicted_output * dt) * pred_error < 0.0f)
        //    //    predicted_output = 1.0f / f3 * pred_error / dt;

        float pred_deriv = (predicted_output - output_shift) / dt;
        output_acc = pred_deriv;

    }
    //    // now let's get more accurate predictions of output_acc
    //    float cout = vel_c->eval(mdl, output_vel, output_acc, dt);
    //    vel_c->already_preupdated = true;
    //    predicted_aoa = predict_aoa(mdl, cout, dt);
    //    float pred_delta_aoa = predicted_aoa - cur_aoa;
    //    // compare pred_delta_aoa to desired shift and change output_shift accordingly
    //    if ((pred_delta_aoa * des_delta_aoa > 0.0f) && (fabs(pred_delta_aoa) > fabs(des_delta_aoa)))
    //    {
    //        // we're overshooting
    //        output_shift *= 0.9f * des_delta_aoa / pred_delta_aoa;
    //        output_vel = output_shift + des_aoa_equil;
    //    }
    //    else
    //        break;
    //}
    //
    ////predicted_output = get_output(vel_c, predicted_aoa, target_aoa, dt);
    //pred_error = target_aoa - predicted_aoa;
    //abs_pred_error = fabsf(pred_error);
    //pred_err_sign = copysignf(1.0f, pred_error);
    //nninputs(3, 0) = abs_pred_error;
    //nnoutput = net.eval(nninputs);
    //predicted_output = nnoutput(0, 0) * pred_err_sign *
    //    powf(abs_pred_error, 2.0f / 3.0f);

    //pred_deriv = (predicted_output - output_shift) / dt;
    //output_acc = pred_deriv;

    //if ((target - predicted_aoa) * (target - cur_aoa) < 0.0f)
    //{
        // let's simply make shift_vel zero
        //output_vel = des_aoa_equil;

        //float l_bound = 0.0f;
        //float r_bound = output_shift;
        //// binary dychotomy to prevent overshooting
        //for (int i = 0; i < AOAPCITER; i++)
        //{
        //    float m_shift = 0.5f * (l_bound + r_bound);
        //    output_vel = m_shift + des_aoa_equil;
        //    float cout = vel_c->eval(mdl, output_vel, output_acc, dt);
        //    vel_c->already_preupdated = true;
        //    predicted_aoa = predict_aoa(mdl, cout, dt);
        //    if ((target - predicted_aoa) * (target - cur_aoa) < 0.0f)
        //    {
        //        // shift vel too big
        //        r_bound = (r_bound + l_bound) * 0.5f;
        //    }
        //    else
        //    {
        //        // we can increase it
        //        l_bound = (r_bound + l_bound) * 0.5f;
        //    }
        //}
        //// update output_acc
        //predicted_output = get_output(vel_c, predicted_aoa, target_aoa, dt);
        //pred_deriv = (predicted_output - output_shift) / dt;
        //output_acc = pred_deriv;
    //}

    return vel_c->eval(mdl, output_vel, 0.0f, dt);
}

__device__ __host__ void aoa_ctrl::preupdate(pitch_model *mdl)
{
    update_pars(mdl);
    already_preupdated = true;
}

//__device__ __host__ float aoa_ctrl::get_output(ang_vel_p *vel_c, float cur_aoa, 
//    float des_aoa, float dt)
//{
//    float error = des_aoa - cur_aoa;
//    float k = params(0, 0);
//    float p = params(1, 0);
//    float x = powf(fabsf(error) / k, 1.0f / p);
//    if (x <= dt)
//        return 0.1f * error / dt;
//    else
//        return -copysignf(1.0f, error) * k * (powf(x - dt, p) - powf(x, p)) / dt;
//}