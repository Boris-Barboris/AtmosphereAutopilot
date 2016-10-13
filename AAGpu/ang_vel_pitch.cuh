#pragma once

#include "aircraftmodel.cuh"

// Pitch ang vel controller
struct ang_vel_p
{
    // tunable parameters
    float max_v_construction;
    float max_aoa;
    //float max_g;
    float quadr_Kp;

    bool moderate_aoa;
    //bool moderate_g;
    bool stable;
    bool already_preupdated;

    float res_max_aoa;
    float res_min_aoa;
    float res_equilibr_v_upper;
    float res_equilibr_v_lower;
    float max_input_aoa;
    float min_input_aoa;
    float max_input_v;
    float min_input_v;
    // note g-force moderation is omitted for performace reasons
    //float max_g_aoa;
    //float min_g_aoa;
    //float max_g_v;
    //float min_g_v;
    float max_aoa_v;
    float min_aoa_v;
    
    float kacc_quadr;

    float target_vel;

    __device__ float eval(pitch_model *mdl, float target, float target_deriv, float dt);
    __device__ void preupdatev(pitch_model *mdl);
};