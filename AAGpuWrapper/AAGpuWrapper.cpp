// This is the main DLL file.

#include "AAGpuWrapper.h"
#include "AAGpu.h"

namespace AAGpuWrapper
{

    void RawModelExperiment::execute()
    {
        int points_count = Math::Ceiling(experiment_length / dt) + 1;
        
        std::array<float, 3> rot_model = 
            { pitchRotModel[0], pitchRotModel[1], pitchRotModel[2] };
        std::array<float, 3> lift_model =
            { pitchLiftModel[0], pitchLiftModel[1], pitchLiftModel[2] };
        std::array<float, 2> drag_model =
            { dragModel[0], dragModel[1] };

        bool aero_model = aerodynamics == AeroModel::FARAero ? true : false;

        std::vector<float> 
            out_angvel(points_count), 
            out_aoa(points_count), 
            out_acc(points_count), 
            out_csurf(points_count), 
            out_input(points_count);

        raw_prototype *exec_func;
        if (computeHost == ExecutionHost::CPU)
            exec_func = raw_execute_cpu;
        else
            exec_func = raw_execute;

        exec_func(
            dt, 
            points_count-1, 
            MOI, 
            mass, 
            sas, 
            rot_model,
            lift_model, 
            drag_model, 
            aero_model, 
            startVel, 
            keepSpeed,
            control,
            out_angvel, 
            out_aoa, 
            out_acc, 
            out_csurf, 
            out_input);

        timePoints = gcnew List<Single>(points_count);
        angVelHistory = gcnew List<Single>(points_count);
        AOAHistory = gcnew List<Single>(points_count);
        angAccHistory = gcnew List<Single>(points_count);
        csurfHistory = gcnew List<Single>(points_count);
        inputHistory = gcnew List<Single>(points_count);

        for (int i = 0; i < points_count; i++)
        {
            timePoints->Add(0.0f + dt * i);
            angVelHistory->Add(out_angvel[i]);
            AOAHistory->Add(out_aoa[i]);
            angAccHistory->Add(out_acc[i]);
            csurfHistory->Add(out_csurf[i]);
            inputHistory->Add(out_input[i]);
        }
    }



    void AoAEvalExperiment::execute()
    {
        int points_count = Math::Ceiling(experiment_length / dt) + 1;

        std::array<float, 3> rot_model =
        { pitchRotModel[0], pitchRotModel[1], pitchRotModel[2] };
        std::array<float, 3> lift_model =
        { pitchLiftModel[0], pitchLiftModel[1], pitchLiftModel[2] };
        std::array<float, 2> drag_model =
        { dragModel[0], dragModel[1] };
        std::array<float, 2> aoa_params =
        { AoA_params[0], AoA_params[1] };

        bool aero_model = aerodynamics == AeroModel::FARAero ? true : false;

        std::vector<float>
            out_angvel(points_count),
            out_aoa(points_count),
            out_acc(points_count),
            out_csurf(points_count),
            out_input(points_count),
            out_outvel(points_count);

        aoa_eval_prototype *exec_func;
        if (computeHost == ExecutionHost::CPU)
            exec_func = aoa_execute_cpu;
        else
            exec_func = aoa_execute;

        exec_func(
            dt,
            points_count - 1,
            MOI,
            mass,
            sas,
            rot_model,
            lift_model,
            drag_model,
            aero_model,
            startVel,
            startAoA,
            keepSpeed,
            control,
            aoa_params,
            out_angvel,
            out_aoa,
            out_acc,
            out_csurf,
            out_input,
            out_outvel);

        timePoints = gcnew List<Single>(points_count);
        angVelHistory = gcnew List<Single>(points_count);
        AOAHistory = gcnew List<Single>(points_count);
        angAccHistory = gcnew List<Single>(points_count);
        csurfHistory = gcnew List<Single>(points_count);
        inputHistory = gcnew List<Single>(points_count);
        outputVelHistory = gcnew List<Single>(points_count);

        for (int i = 0; i < points_count; i++)
        {
            timePoints->Add(0.0f + dt * i);
            angVelHistory->Add(out_angvel[i]);
            AOAHistory->Add(out_aoa[i]);
            angAccHistory->Add(out_acc[i]);
            csurfHistory->Add(out_csurf[i]);
            inputHistory->Add(out_input[i]);
            outputVelHistory->Add(out_outvel[i]);
        }
    }

}