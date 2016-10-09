#include "aircraftmodel.cuh"
#include "math_functions.hpp"
#include "math_constants.h"
#include "vector_utils.h"

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
    float K0 = rot_m.x * 1e-3f * dyn_pressure / moi;
    float K1 = rot_m.y * 1e-3f * dyn_pressure / moi;
    float K2 = rot_m.z * 1e-3f * dyn_pressure / moi;

    pitch_A(0, 0) = - Cl1 / velocity_magn;
    pitch_A(0, 1) = 1.0f;
    pitch_A(0, 2) = - Cl2 / velocity_magn;
    pitch_A(1, 0) = K1;
    if (!aero_model)
    {
        pitch_A(1, 2) = K2;
        pitch_B(1, 0) = sas_torque / moi;
        pitch_C(2, 0) = stock_csurf_spd;
    }
    else
    {
        // FAR
        pitch_A(1, 2) = K2 * (1.0f - dt / far_timeConstant);
        pitch_B(1, 0) = sas_torque / moi + K2 * dt / far_timeConstant;
        pitch_A(2, 2) = -1.0f / far_timeConstant;
        pitch_B(2, 0) = 1.0f / far_timeConstant;
    }
    pitch_C(0, 0) = -(pitch_gravity_acc + Cl0) / velocity_magn;
    pitch_C(1, 0) = K0;

    pitch_A_undelayed.copyFrom(pitch_A);
    pitch_A_undelayed(1, 2) = 0.0f;
    pitch_B_undelayed.copyFrom(pitch_B);
    pitch_B_undelayed(1, 0) = sas_torque / moi + K2;

    // update aoa
    float2 fwd_vector = make_float2(cosf(pitch_angle), sinf(pitch_angle));
    float aoa_angle = acosf(fminf(1.0f, fmaxf(-1.0f, 
        dot(normalize(velocity), fwd_vector))));
    float2 up_vector = make_float2(cosf(up_angle), sinf(up_angle));
    if (dot(velocity, up_vector) > 0.0f)
        aoa_angle = -aoa_angle;
    pitch_tangent = normalize(make_float2(-velocity.y, velocity.x));
    aoa = aoa_angle;
}

__device__ void pitch_model::simulation_step(float dt, float input)
{
    // update_control_states
    if (!aero_model)
        csurf_state = moveto(csurf_state, input, dt * stock_csurf_spd);
    else
        csurf_state_new = moveto(csurf_state, input, dt / far_timeConstant);

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

    float pitch_acc = (float)(pitch_A.rowSlice<1>() * colVec(aoa, ang_vel, csurf_state) + \
        pitch_B(1, 0) * input + pitch_C(1, 0));
    if (aero_model)
        csurf_state = csurf_state_new;
    float rot_delta = ang_vel * dt + 0.5f * dt * dt * pitch_acc;
    pitch_angle += rot_delta;
    ang_vel += dt * pitch_acc;
}