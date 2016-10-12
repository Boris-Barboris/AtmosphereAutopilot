#pragma once

#include "aircraftmodel.cuh"

// Pitch ang acc controller
struct ang_acc_p
{
    // It's pretty much stateless, so well just define static function
    static __device__ float eval(pitch_model *mdl, 
        float des_acc, float dt);
};