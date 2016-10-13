#include "ang_acc_pitch.cuh"


namespace ang_acc_p
{

    __device__ __host__ float eval_ac(pitch_model *mdl, float des_acc, float dt)
    {
        if (aero_model)
        {
            // FAR branch
            float cur_input = mdl->csurf_state;
            matrix<3, 1> cur_state = colVec(mdl->aoa, mdl->ang_vel, cur_input);
            float predicted_acc = (float)(mdl->A.rowSlice<1>() * cur_state) +
                mdl->B(1, 0) * cur_input + mdl->C(1, 0);
            float acc_error = des_acc - predicted_acc;
            float authority = mdl->B(1, 0);
            float new_output = clamp(cur_input + acc_error / authority, -1.0f, 1.0f);
            if (fabsf(new_output - cur_input) * 10.0f < 0.1f)
            {
                authority = mdl->B_undelayed(1, 0);
                new_output = clamp(cur_input + acc_error / authority, -1.0f, 1.0f);
            }
            return new_output;
        }
        else
        {
            // Stock branch
            float cur_input = mdl->csurf_state;
            matrix<2, 1> cur_state = colVec(mdl->aoa, mdl->ang_vel);
            float predicted_acc = (float)(mdl->A_undelayed.rowSlice<1>() * cur_state) +
                mdl->B_undelayed(1, 0) * cur_input + mdl->C(1, 0);
            float acc_error = des_acc - predicted_acc;
            float authority = mdl->B_undelayed(1, 0);
            float new_output = clamp(cur_input + acc_error / authority, -1.0f, 1.0f);
            if (fabsf(new_output - cur_input) / dt > stock_csurf_spd)
            {
                matrix<3, 1> exp_state = colVec(mdl->aoa, mdl->ang_vel,
                    clamp(cur_input + copysignf(dt * stock_csurf_spd,
                        new_output - cur_input), -1.0f, 1.0f));
                cur_input = exp_state(2, 0);
                predicted_acc = (float)(mdl->A.rowSlice<1>() * exp_state) +
                    mdl->B(1, 0) * cur_input + mdl->C(1, 0);
                acc_error = des_acc - predicted_acc;
                authority = mdl->B(1, 0);
                new_output = clamp(cur_input + acc_error / authority, -1.0f, 1.0f);
            }
            return new_output;
        }
    }

}