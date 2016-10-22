#pragma once

#include <array>
#include <vector>

#include "aoa_ctrl_constants.h"

#ifdef AAGPU_EXPORTS
#define AAGPU_EXPORTS_API __declspec(dllexport) 
#else
#define AAGPU_EXPORTS_API __declspec(dllimport) 
#endif

// Raw model evaluation without controllers

typedef void raw_prototype(
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

AAGPU_EXPORTS_API raw_prototype raw_execute;

AAGPU_EXPORTS_API raw_prototype raw_execute_cpu;


// Evaluation under control of AoA controller

typedef void aoa_eval_prototype(
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
    float start_aoa,
    bool keep_speed,
    float target_aoa,
    const std::array<float, AOAPARS> &aoa_params,
    const std::array<std::tuple<float, float>, AOAINPUTS> &input_norms,
    const std::array<std::tuple<float, float>, AOAOUTPUTS> &output_norms,
    std::vector<float> &out_angvel,
    std::vector<float> &out_aoa,
    std::vector<float> &out_acc,
    std::vector<float> &out_csurf,
    std::vector<float> &out_input,
    std::vector<float> &out_vel_output);

AAGPU_EXPORTS_API aoa_eval_prototype aoa_execute;

AAGPU_EXPORTS_API aoa_eval_prototype aoa_execute_cpu;


// AoA PSO single model optimization
typedef void (__stdcall *report_dlg)(int epoch, float value, std::array<float, AOAPARS> params);

#define PARTICLEBLOCK 256

AAGPU_EXPORTS_API bool start_aoa_pso(
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
    const std::array<std::tuple<float, float>, AOAINPUTS> &input_norms,
    const std::array<std::tuple<float, float>, AOAOUTPUTS> &output_norms,
    int prtcl_blocks,
    float w,
    float c1,
    float c2,
    float initial_span,
    int aoa_divisions,
    const std::array<float, 4> &exper_weights,
    report_dlg repotrer);

AAGPU_EXPORTS_API void stop_aoa_pso();