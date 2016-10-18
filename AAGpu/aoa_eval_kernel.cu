#include "AAGpu.h"

#include "aoa_ctrl.cuh"


struct thread_context
{
    pitch_model mdl;
    ang_vel_p vel_c;
    aoa_ctrl aoa_c;
};

#define AOAKERNELGPU
#include "aoa_eval_kernel_common.cuh"

#undef AOAKERNELGPU
#include "aoa_eval_kernel_common.cuh"