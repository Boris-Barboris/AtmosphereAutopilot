#include "AAGpu.h"

#include "ang_vel_pitch.cuh"


struct thread_context
{
    pitch_model mdl;
    ang_vel_p vel_c;
};

#define RAWKERNELGPU
#include "raw_kernel_common.cuh"

#undef RAWKERNELGPU
#include "raw_kernel_common.cuh"