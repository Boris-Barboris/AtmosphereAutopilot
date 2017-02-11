#pragma once

#include "aircraftmodel.cuh"
#include "ang_vel_pitch.cuh"
//#include "ann.cuh"
#include "aoa_ctrl_constants.h"

// Pitch AoA controller
struct aoa_ctrl
{
    // tunable parameters
    matrix<AOALINPARAMS, 1> params;
    
    // state
    float output_vel;
    float output_acc;
    float predicted_aoa;
    float predicted_eq_v;
    float predicted_output;
    float target_aoa;
    float cur_aoa_equilibr;     // equilibrium angular velocity to stay on current AoA

    bool already_preupdated;

    // Initializer
    inline __device__ __host__ void zero_init()
    {
        params = matrix<AOALINPARAMS, 1>();
        output_vel = 0.0;
        output_acc = 0.0;
        predicted_aoa = 0.0;
        predicted_eq_v = 0.0;
        predicted_output = 0.0;
        target_aoa = 0.0;
        cur_aoa_equilibr = 0.0;
        already_preupdated = false;
    }

    __device__ __host__ float eval(pitch_model *mdl, ang_vel_p *vel_c, float target, 
        float target_deriv, float dt);
    __device__ __host__ void preupdate(pitch_model *mdl);
    __device__ __host__ static matrix<2, 1> get_equlibr(pitch_model *mdl, float aoa);

private:
    __device__ __host__ void update_pars(pitch_model *mdl);
    /*__device__ __host__ float get_output(ang_vel_p *vel_c, float cur_aoa, 
        float des_aoa, float dt);*/
};