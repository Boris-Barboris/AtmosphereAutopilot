#pragma once

#include "matrix.cuh"

extern __constant__ float   density;
extern __constant__ bool    aero_model;         // false - stock, true - FAR
extern __constant__ bool    spd_const;
extern __constant__ float   gravity;
extern __constant__ float   far_timeConstant;
extern __constant__ float   stock_csurf_spd;

// Pitch dynamics model
// need 8-byte alignment because we use float2
struct __align__(8) pitch_model
{
    float2 position;                    // 8
    float2 velocity;                    // 16
    float pitch_angle;                  // 20
    float ang_vel;                      // 24
    float csurf_state;                  // 28
    float csurf_state_new;              // 32
    float aoa;                          // 36
    float moi;                          // 40
    float mass;                         // 44
    float3 rot_m;                       // 56
    float3 lift_m;                      // 68
    float2 drag_m;                      // 76
    float sas_torque;                   // 80

    // matrixes
    matrix<3, 3> pitch_A;               // 116
    matrix<3, 1> pitch_B;               // 128
    matrix<3, 1> pitch_C;               // 140
    matrix<2, 2> pitch_A_undelayed;     // 156
    matrix<2, 1> pitch_B_undelayed;     // 164

    // Methods
    __device__ void preupdate(float dt);
    __device__ void simulation_step(float dt, float input);

    // Misc
    float dyn_pressure;                 // 168
    float2 pitch_tangent;               // 176
    float ang_acc;                      // 180
    
    //int _stride;                        // 184
};

__inline__ __device__ float clamp(float val, float lower, float upper)
{
    return fmaxf(lower, fminf(upper, val));
}

__inline__ __device__ float moveto(float cur_state, float des_state, float max_delta)
{
    return cur_state + clamp(des_state - cur_state, -max_delta, max_delta);
}

__inline__ __device__ float moveto_far(float cur_state, float des_state, float time_div)
{
    float error = des_state - cur_state;
    if (fabsf(error) * 10.0f >= 0.1f)
        return cur_state + clamp(error * time_div, 
            -fabsf(0.6f * error), fabsf(0.6f * error));
    else
        return des_state;
}