#include "ang_vel_pitch.cuh"
#include "ang_acc_pitch.cuh"

#include "math_constants.h"

__device__ __host__ static void vel_update_pars(ang_vel_p &obj, pitch_model *mdl)
{
    if (obj.already_preupdated)
    {
        obj.already_preupdated = false;
        return;
    }

    float rad_max_aoa = obj.max_aoa;
    obj.res_max_aoa = 100.0f;
    obj.res_min_aoa = -100.0f;
    obj.res_equilibr_v_upper = 0.0f;
    obj.res_equilibr_v_lower = 0.0f;

    float cur_aoa = mdl->aoa;
    //float abs_cur_aoa = fabsf(cur_aoa);
    
    // let's omit all moderated stuff
    if (obj.moderate_aoa)
    {
        // get equilibrium aoa and angular_v for 1.0 input
        matrix<2, 2> eq_A;
        eq_A(0, 0) = mdl->A(0, 0);
        eq_A(0, 1) = mdl->A(0, 1);
        eq_A(1, 0) = mdl->A(1, 0);
        eq_A(1, 1) = 0.0f;        
        matrix<2, 1> eq_B = colVec(
            -(mdl->A(0, 2) + mdl->B(0, 0) + mdl->C(0, 0)),
            -(mdl->A(1, 2) + mdl->B(1, 0) + mdl->C(1, 0)));
        matrix<2, 1> eq_x = eq_A / eq_B;
        if (eq_x(0, 0) < 0.0f)
        {
            obj.stable = false;
            obj.min_input_aoa = 0.6f * eq_x(0, 0);
            obj.min_input_v = 0.6f * eq_x(1, 0);
        }
        else
        {
            obj.stable = true;
            obj.max_input_aoa = eq_x(0, 0);
            obj.max_input_v = eq_x(1, 0);
        }

        // get equilibrium aoa and angular_v for -1.0 input
        eq_B = colVec(
            mdl->A(0, 2) + mdl->B(0, 0) - mdl->C(0, 0),
            mdl->A(1, 2) + mdl->B(1, 0) - mdl->C(1, 0));
        eq_x = eq_A / eq_B;
        if (eq_x(0, 0) > 0.0)
        {
            obj.max_input_aoa = 0.6f * eq_x(0, 0);
            obj.max_input_v = 0.6f * eq_x(1, 0);
        }
        else
        {
            obj.min_input_aoa = eq_x(0, 0);
            obj.min_input_v = eq_x(1, 0);
        }

        // max aoa section
        eq_A(0, 0) = mdl->A(0, 1);
        eq_A(0, 1) = mdl->A(0, 2) + mdl->B(0, 0);
        eq_A(1, 0) = mdl->A(1, 1);
        eq_A(1, 1) = mdl->A(1, 2) + mdl->B(1, 0);
        eq_B(0, 0) = -(mdl->A(0, 0) * rad_max_aoa + mdl->C(0, 0));
        eq_B(1, 0) = -(mdl->A(1, 0) * rad_max_aoa + mdl->C(1, 0));
        eq_x = eq_A / eq_B;
        obj.max_aoa_v = eq_x(0, 0);
        eq_B(0, 0) = mdl->A(0, 0) * rad_max_aoa - mdl->C(0, 0);
        eq_B(1, 0) = mdl->A(1, 0) * rad_max_aoa - mdl->C(1, 0);
        eq_x = eq_A / eq_B;
        obj.min_aoa_v = eq_x(0, 0);

        // let's apply moderation with controllability region
        if (obj.max_input_aoa < obj.res_max_aoa)
        {
            obj.res_max_aoa = obj.max_input_aoa;
            obj.res_equilibr_v_upper = obj.max_input_v;
        }
        if (obj.min_input_aoa > obj.res_min_aoa)
        {
            obj.res_min_aoa = obj.min_input_aoa;
            obj.res_equilibr_v_lower = obj.min_input_v;
        }

        // apply simple AoA moderation
        if (rad_max_aoa < obj.res_max_aoa)
        {
            obj.res_max_aoa = rad_max_aoa;
            obj.res_equilibr_v_upper = obj.max_aoa_v;
        }
        if (-rad_max_aoa > obj.res_min_aoa)
        {
            obj.res_min_aoa = -rad_max_aoa;
            obj.res_equilibr_v_lower = obj.min_aoa_v;
        }
    }

    // G-moderation omitted

    // kacc_quadr section
    float kacc;
    if (aero_model)
        kacc = obj.quadr_Kp * (mdl->A(1, 2) * (0.5f / far_timeConstant) + mdl->B(1, 0)); // FAR
    else
        kacc = obj.quadr_Kp * (mdl->A(1, 2) * mdl->C(2, 0) + mdl->B(1, 0)); // stock
    obj.kacc_quadr = 0.9f * fabsf(kacc);
}

__device__ __host__ static float get_desired_acc(ang_vel_p &obj, pitch_model *mdl, float des_v,
    float target_deriv, float dt)
{
    float cur_v = mdl->ang_vel;
    float v_error = cur_v - des_v;
    float d = target_deriv;
    float k = copysignf(obj.kacc_quadr, v_error + d * dt);
    if (k == 0.0f)
        return target_deriv;

    float b_s = d + 2.0f * k * dt;
    float a_s = k;
    float c_s = k * dt * dt + d * d / 4.0f / k - v_error;
    float D_s = b_s * b_s - 4.0f * a_s * c_s;
    D_s = fmaxf(D_s, 0.0f);
    float s1 = (-b_s + sqrtf(D_s)) / 2.0f / a_s;
    float s2 = (-b_s - sqrtf(D_s)) / 2.0f / a_s;
    float s = fmaxf(s1, s2);
    float b = d * s + d * d / 4.0f /  k;

    if (s <= 0.0f)
        //return  (d * dt - v_error) / dt;
        return  -v_error / dt;
    else
        return ((k * s * s + b) - v_error) / dt;
    //return 0.0f;
}

__device__ __host__ float ang_vel_p::eval(pitch_model *mdl, float target, float target_deriv,
    float dt)
{
    target_vel = target;
    vel_update_pars(*this, mdl);
    float acc_unconst = get_desired_acc(*this, mdl, target, target_deriv, dt);
    float acc_constrained = get_desired_acc(*this, mdl,
        copysignf(max_v_construction, target), 0.0f, dt);
    float output_acc;
    if (acc_constrained * copysignf(1.0f, target) < acc_unconst * copysignf(1.0f, target))
        output_acc = acc_constrained;
    else
        output_acc = acc_unconst;
    return ang_acc_p::eval_ac(mdl, output_acc, dt);
}

__device__ __host__ void ang_vel_p::preupdatev(pitch_model *mdl)
{
    vel_update_pars(*this, mdl);
    already_preupdated = true;
}