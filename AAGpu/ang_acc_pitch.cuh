#pragma once

#include "aircraftmodel.cuh"

namespace ang_acc_p
{
    // It's pretty much stateless, so well just define static function
    __device__ __host__ float eval_ac(pitch_model *mdl, float des_acc, float dt);
};