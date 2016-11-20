// This is the main DLL file.

#include "AAGpuWrapper.h"
#include "AAGpu.h"

using namespace System::Runtime::InteropServices;

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

    void AoAEvalExperiment::randomize_params()
    {
        Random ^rng = gcnew Random();
        for (int i = 0; i < AOAPARS; i++)
        {
            float val = (float)(rng->NextDouble() - 0.5);
            AoA_params->Add(val);
        }
    }

    void AoAEvalExperiment::init_normals()
    {
        InputLowerBounds = gcnew List<Single>();
        InputUpperBounds = gcnew List<Single>();
        OutputLowerBounds = gcnew List<Single>();
        OutputUpperBounds = gcnew List<Single>();

        InputLowerBounds->Add(0.0f);
        InputLowerBounds->Add(0.0f);
        InputLowerBounds->Add(-0.01f);
        InputLowerBounds->Add(0.0f);

        InputUpperBounds->Add(10.0f);
        InputUpperBounds->Add(2.0f);
        InputUpperBounds->Add(0.01f);
        InputUpperBounds->Add(0.5f);

        OutputLowerBounds->Add(-0.1f);
        OutputUpperBounds->Add(10.0f);
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
        std::array<float, AOAPARS> aoa_params;
        for (int i = 0; i < AOAPARS; i++)
            aoa_params[i] = AoA_params[i];
        std::array<std::tuple<float, float>, AOAINPUTS> input_norms;
        for (int i = 0; i < AOAINPUTS; i++)
            input_norms[i] = std::make_tuple(InputLowerBounds[i], InputUpperBounds[i]);
        std::array<std::tuple<float, float>, AOAOUTPUTS> output_norms;
        for (int i = 0; i < AOAOUTPUTS; i++)
            output_norms[i] = std::make_tuple(OutputLowerBounds[i], OutputUpperBounds[i]);

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
            input_norms,
            output_norms,
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




    void AoAPsoOptimization::init_normals()
    {
        InputLowerBounds = gcnew List<Single>();
        InputUpperBounds = gcnew List<Single>();
        OutputLowerBounds = gcnew List<Single>();
        OutputUpperBounds = gcnew List<Single>();

        InputLowerBounds->Add(0.0f);
        InputLowerBounds->Add(0.0f);
        InputLowerBounds->Add(-0.1f);
        InputLowerBounds->Add(0.0f);

        InputUpperBounds->Add(10.0f);
        InputUpperBounds->Add(2.0f);
        InputUpperBounds->Add(0.1f);
        InputUpperBounds->Add(0.5f);

        OutputLowerBounds->Add(-0.1f);
        OutputUpperBounds->Add(10.0f);
    }

    delegate void native_reporter(int epoch, float val, std::array<float, AOAPARS> bp);

    static GCHandle reporter_handle1;
    static bool dlg_init = false;
    static report_dlg pinned_report_func_ptr;

    static void report_native(int epoch, float val, std::array<float, AOAPARS> bp)
    {
        List<float> ^l = gcnew List<float>(AOAPARS);
        for (int i = 0; i < AOAPARS; i++)
            l->Add(bp[i]);
        AoAPsoOptimization::report_dlg_stat(epoch, val, l);
    }

    void AoAPsoOptimization::init_delegate()
    {
        if (dlg_init)
            reporter_handle1.Free();
        native_reporter ^native_dlg = gcnew native_reporter(report_native);
        reporter_handle1 = GCHandle::Alloc(native_dlg);
        IntPtr ip = Marshal::GetFunctionPointerForDelegate(native_dlg);
        pinned_report_func_ptr = static_cast<report_dlg>(ip.ToPointer());
        dlg_init = true;
    }

    bool AoAPsoOptimization::start()
    {
        int points_count = Math::Ceiling(experiment_length / dt) + 1;

        std::array<float, 3> rot_model =
        { pitchRotModel[0], pitchRotModel[1], pitchRotModel[2] };
        std::array<float, 3> lift_model =
        { pitchLiftModel[0], pitchLiftModel[1], pitchLiftModel[2] };
        std::array<float, 2> drag_model =
        { dragModel[0], dragModel[1] };

        pitch_model_params base_model;
        base_model.mass = mass;
        base_model.moi = MOI;
        base_model.sas = sas;
        base_model.rot_model = rot_model;
        base_model.lift_model = lift_model;
        base_model.drag_model = drag_model;

        std::array<float, 4> weights =
        { ExperimentWeights[0], ExperimentWeights[1], ExperimentWeights[2],
          ExperimentWeights[3]};
        std::array<std::tuple<float, float>, AOAINPUTS> input_norms;
        for (int i = 0; i < AOAINPUTS; i++)
            input_norms[i] = std::make_tuple(InputLowerBounds[i], InputUpperBounds[i]);
        std::array<std::tuple<float, float>, AOAOUTPUTS> output_norms;
        for (int i = 0; i < AOAOUTPUTS; i++)
            output_norms[i] = std::make_tuple(OutputLowerBounds[i], OutputUpperBounds[i]);

        bool aero_model = aerodynamics == AeroModel::FARAero ? true : false;

        // create corpus
        auto corpus = generate_corpus(base_model, moi_steps, moi_min, moi_max,
            t_ratio_steps, ratio_min, ratio_max, cl2_steps, cl2_min, cl2_max);

        return start_aoa_pso(
            dt,
            points_count - 1,
            corpus,
            aero_model,
            startVel,
            keepSpeed,
            input_norms,
            output_norms,
            threadBlocks,
            w,
            c1,
            c2,
            span,
            aoa_divisions,
            weights,
            pinned_report_func_ptr);
    }

    void AoAPsoOptimization::stop()
    {
        stop_aoa_pso();
    }

}