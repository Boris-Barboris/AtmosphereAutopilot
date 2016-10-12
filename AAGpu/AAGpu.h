#pragma once

#include <array>
#include <vector>

#ifdef AAGPU_EXPORTS
#define AAGPU_EXPORTS_API __declspec(dllexport) 
#else
#define AAGPU_EXPORTS_API __declspec(dllimport) 
#endif


AAGPU_EXPORTS_API void __cdecl raw_execute(
    float dt,
    int step_count,
    float moi,
    float mass,
    float sas,
    const std::array<float, 3> &rot_model,
    const std::array<float, 3> &lift_model,
    const std::array<float, 2> &drag_model,
    bool aero_model,
    float start_vel,
    bool keep_speed,
    float input,
    std::vector<float> &out_angvel,
    std::vector<float> &out_aoa,
    std::vector<float> &out_acc,
    std::vector<float> &out_csurf,
    std::vector<float> &out_input);
