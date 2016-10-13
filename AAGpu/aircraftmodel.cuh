#pragma once

#include "matrix.cuh"
#include <math.h>
#include "vector_utils.cuh"

extern __constant__ float   d_density;
extern float h_density;
extern __constant__ bool    d_aero_model;         // false - stock, true - FAR
extern bool h_aero_model;
extern __constant__ bool    d_spd_const;
extern bool h_spd_const;
extern __constant__ float   d_gravity;
extern float h_gravity;
extern __constant__ float   d_far_timeConstant;
extern float h_far_timeConstant;
extern __constant__ float   d_stock_csurf_spd;
extern float h_stock_csurf_spd;

#ifdef __CUDA_ARCH__
#define density d_density    
#define aero_model d_aero_model
#define spd_const d_spd_const
#define gravity d_gravity
#define far_timeConstant d_far_timeConstant
#define stock_csurf_spd d_stock_csurf_spd
#else
#define density h_density    
#define aero_model h_aero_model
#define spd_const h_spd_const
#define gravity h_gravity
#define far_timeConstant h_far_timeConstant
#define stock_csurf_spd h_stock_csurf_spd
#endif // __CUDA_ARCH__


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
    matrix<3, 3> A;               // 116
    matrix<3, 1> B;               // 128
    matrix<3, 1> C;               // 140
    matrix<2, 2> A_undelayed;     // 156
    matrix<2, 1> B_undelayed;     // 164

    // Methods
    __device__ __host__ void preupdate(float dt);
    __device__ __host__ void simulation_step(float dt, float input);

    // Initializer
    inline __device__ __host__ void zero_init()
    {
        position = float2_zero;
        velocity = float2_zero;
        pitch_angle = 0.0f;
        ang_vel = 0.0f;
        csurf_state = 0.0f;
        csurf_state_new = 0.0f;
        aoa = 0.0f;
        moi = 0.0f;
        mass = 0.0f;
        rot_m = float3_zero;
        lift_m = float3_zero;
        drag_m = float2_zero;
        sas_torque = 0.0f;
        dyn_pressure = 0.0f;
        pitch_tangent = float2_zero;
        ang_acc = 0.0f;
        A = matrix<3, 3>();
        B = matrix<3, 1>();
        C = matrix<3, 1>();
        A_undelayed = matrix<2, 2>();
        B_undelayed = matrix<2, 1>();
    }

    // Misc
    float dyn_pressure;                 // 168
    float2 pitch_tangent;               // 176
    float ang_acc;                      // 180
    
    //int _stride;                        // 184
};

inline __device__ __host__ 
float clamp(float val, float lower, float upper)
{
    return fmaxf(lower, fminf(upper, val));
}

inline __device__ __host__ 
float moveto(float cur_state, float des_state, float max_delta)
{
    return cur_state + clamp(des_state - cur_state, -max_delta, max_delta);
}

inline __device__ __host__
float moveto_far(float cur_state, float des_state, float time_div, bool &collapse)
{
    float error = des_state - cur_state;
    if (fabsf(error) * 10.0f >= 0.1f)
    {
        collapse = false;
        return cur_state + clamp(error * time_div,
            -fabsf(0.6f * error), fabsf(0.6f * error));
    }
    else
    {
        collapse = true;
        return des_state;
    }
}