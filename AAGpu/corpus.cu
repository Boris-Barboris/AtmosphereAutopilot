#include "AAGpu.h"

std::vector<pitch_model_params> generate_corpus(
    const pitch_model_params &base_model,
    int moi_steps,
    float moi_min,
    float moi_max,
    int t_ratio_steps,
    float ratio_min,
    float ratio_max,
    int cl2_steps,
    float cl2_min,
    float cl2_max)
{
    std::vector<pitch_model_params> output;
    for (int i = 0; i < moi_steps; i++)
        for (int j = 0; j < t_ratio_steps; j++)
            for (int k = 0; k < cl2_steps; k++)
            {
                float moi = moi_min + i * (moi_max - moi_min) / 
                    (float)(moi_steps - 1);
                float ratio = ratio_min + j * (ratio_max - ratio_min) /
                    (float)(t_ratio_steps - 1);
                float cl2 = cl2_min + k * (cl2_max - cl2_min) /
                    (float)(cl2_steps - 1);
                pitch_model_params model = base_model;
                model.moi = moi;
                model.rot_model[1] = ratio * model.rot_model[2];
                model.lift_model[2] = cl2;
                output.push_back(model);
            }
    return output;
}