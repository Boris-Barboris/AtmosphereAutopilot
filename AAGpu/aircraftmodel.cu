#include "aircraftmodel.cuh"
#include "math_functions.hpp"
#include "math_constants.h"
#include "vector_utils.cuh"

__constant__ float   density = 1.0f;
__constant__ bool    aero_model = false;         // false - stock, true - FAR
__constant__ bool    spd_const = true;
__constant__ float   gravity = -9.8f;
__constant__ float   far_timeConstant = 0.25f;
__constant__ float   stock_csurf_spd = 2.0f;



__device__ void pitch_model::preupdate(float dt)
{
    float veldot = dot(velocity, velocity);
    float velocity_magn = sqrtf(veldot);
    dyn_pressure = density * veldot;

    // update_dynamics
    float up_angle = pitch_angle + CUDART_PIO2_F;
    float pitch_gravity_acc = gravity * sinf(up_angle);

    // update_pitch_matrix
    float Cl0 = lift_m.x * 1e-3f * dyn_pressure / mass;
    float Cl1 = lift_m.y * 1e-3f * dyn_pressure / mass;
    float Cl2 = lift_m.z * 1e-3f * dyn_pressure / mass;
    float K0 = rot_m.x * 1e-2f * dyn_pressure / moi;
    float K1 = rot_m.y * 1e-2f * dyn_pressure / moi;
    float K2 = rot_m.z * 1e-2f * dyn_pressure / moi;

    A(0, 0) = - Cl1 / velocity_magn;
    A(0, 1) = 1.0f;
    A(0, 2) = - Cl2 / velocity_magn;
    A(1, 0) = K1;
    if (!aero_model)
    {
        A(1, 2) = K2;
        B(1, 0) = sas_torque / moi;
        C(2, 0) = stock_csurf_spd;
    }
    else
    {
        // FAR
        A(1, 2) = K2 * (1.0f - dt / far_timeConstant);
        B(1, 0) = sas_torque / moi + K2 * dt / far_timeConstant;
        A(2, 2) = -1.0f / far_timeConstant;
        B(2, 0) = 1.0f / far_timeConstant;
    }
    C(0, 0) = -(pitch_gravity_acc + Cl0) / velocity_magn;
    C(1, 0) = K0;

    A_undelayed.copyFrom(A);
    A_undelayed(1, 2) = 0.0f;
    B_undelayed.copyFrom(B);
    B_undelayed(1, 0) = sas_torque / moi + K2;

    // update aoa
    float2 fwd_vector = make_float2(cosf(pitch_angle), sinf(pitch_angle));
    float rightv = fminf(1.0f, fmaxf(-1.0f, 
        hypercross(normalize(velocity), fwd_vector)));
    float asin = asinf(rightv);
    if (dot(fwd_vector, velocity) >= 0.0f)
        aoa = asin;
    else
        aoa = CUDART_PI - asin;
    pitch_tangent = normalize(make_float2(-velocity.y, velocity.x));
}

__device__ void pitch_model::simulation_step(float dt, float input)
{
    // update_control_states
    if (!aero_model)
        csurf_state = moveto(csurf_state, input, dt * stock_csurf_spd);
    else
        csurf_state_new = moveto_far(csurf_state, input, dt / far_timeConstant);

    // integrate_dynamics

    // translation section
    float2 acc = make_float2(0.0f, gravity);
    float speed = magn(velocity);

    float dk0 = drag_m.x * 1e-3f * dyn_pressure / mass;
    float dk1 = drag_m.y * 1e-3f * dyn_pressure / mass;
    float2 drag_acc = - (velocity / speed * (dk0 + dk1 * aoa * aoa));

    float Cl0 = lift_m.x * 1e-3 * dyn_pressure / mass;
    float Cl1 = lift_m.y * 1e-3 * dyn_pressure / mass;
    float Cl2 = lift_m.z * 1e-3 * dyn_pressure / mass;
    float2 pitch_lift_acc = pitch_tangent * (Cl0 + Cl1 * aoa + Cl2 * csurf_state);

    acc = acc + drag_acc + pitch_lift_acc;
    position = position + velocity * dt + 0.5f * acc * (dt * dt);
    velocity = velocity + acc * dt;
    if (spd_const)
        velocity = normalize(velocity) * speed;

    // rotation section

    ang_acc = (float)(A.rowSlice<1>() * colVec(aoa, ang_vel, csurf_state)) + 
        B(1, 0) * input + C(1, 0);
    if (aero_model)
        csurf_state = csurf_state_new;
    pitch_angle += ang_vel * dt + 0.5f * dt * dt * ang_acc;
    ang_vel += dt * ang_acc;
}