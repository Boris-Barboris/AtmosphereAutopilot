#pragma once

using namespace System;
using namespace System::IO;
using namespace System::Collections::Generic;
using namespace System::ComponentModel;
using namespace System::ComponentModel::Design;

namespace AAGpuWrapper
{
    public enum class AeroModel
    {
        StockAero,
        FARAero
    };

    public enum class ExecutionHost
    {
        CPU,
        GPU,
        Mixed
    };

    public ref class RawModelExperiment
    {
    public:
        RawModelExperiment()
        {
            dt = 0.05f;
            experiment_length = 5.0f;
            MOI = 165.0f;
            mass = 14.0f;
            sas = 15.0;
            pitchRotModel = gcnew List<Single>();
            pitchRotModel->Add(0.0f);
            pitchRotModel->Add(-1.0f);
            pitchRotModel->Add(1.15f);
            pitchLiftModel = gcnew List<Single>();
            pitchLiftModel->Add(0.0f);
            pitchLiftModel->Add(60.0f);
            pitchLiftModel->Add(-0.25f);
            dragModel = gcnew List<Single>();
            dragModel->Add(1.0f);
            dragModel->Add(20.0f);
            aerodynamics = AeroModel::StockAero;
            startVel = 200.0f;
            keepSpeed = false;
            control = 0.0f;
            computeHost = ExecutionHost::CPU;
        }

        [CategoryAttribute("Time parameters")]
        [DisplayNameAttribute("Time step, sec")]
        property Single dt;

        [CategoryAttribute("Time parameters")]
        [DisplayNameAttribute("Sim length, sec")]
        property Single experiment_length;

        [CategoryAttribute("Craft parameters")]
        [DisplayNameAttribute("Pitch MOI")]
        property Single MOI;

        [CategoryAttribute("Craft parameters")]
        [DisplayNameAttribute("Craft mass")]
        property Single mass;

        [CategoryAttribute("Craft parameters")]
        [DisplayNameAttribute("SAS torque")]
        property Single sas;

        [CategoryAttribute("Craft parameters")]
        [DisplayNameAttribute("Pitch rot model")]
        property List<Single> ^pitchRotModel;

        [CategoryAttribute("Craft parameters")]
        [DisplayNameAttribute("Pitch lift model")]
        property List<Single> ^pitchLiftModel;

        [CategoryAttribute("Craft parameters")]
        [DisplayNameAttribute("Drag model")]
        property List<Single> ^dragModel;

        [CategoryAttribute("Global parameters")]
        [DisplayNameAttribute("Aero model")]
        property AeroModel aerodynamics;

        [CategoryAttribute("Global parameters")]
        [DisplayNameAttribute("Start speed")]
        property Single startVel;

        [CategoryAttribute("Global parameters")]
        [DisplayNameAttribute("Keep speed")]
        property Boolean keepSpeed;

        [CategoryAttribute("Control")]
        [DisplayNameAttribute("Pitch input")]
        property Single control;

        [CategoryAttribute("Computation")]
        [DisplayNameAttribute("Host")]
        property ExecutionHost computeHost;

        virtual void execute();

        // Results

        [Browsable(false)]
        property List<Single> ^timePoints;

        [Browsable(false)]
        property List<Single> ^angVelHistory;

        [Browsable(false)]
        property List<Single> ^AOAHistory;

        [Browsable(false)]
        property List<Single> ^angAccHistory;

        [Browsable(false)]
        property List<Single> ^csurfHistory;

        [Browsable(false)]
        property List<Single> ^inputHistory;
    };




    public ref class AoAEvalExperiment : public RawModelExperiment
    {
    public:
        AoAEvalExperiment(): RawModelExperiment()
        {
            startAoA = 0.0f;
            AoA_params = gcnew List<Single>();
            InputLowerBounds = gcnew List<Single>();
            InputUpperBounds = gcnew List<Single>();
            OutputLowerBounds = gcnew List<Single>();
            OutputUpperBounds = gcnew List<Single>();
            randomize_params();
            init_normals();
            //AoA_params->Add(1.0f);
            //AoA_params->Add(3.0f);
        }

        [CategoryAttribute("Global parameters")]
        [DisplayNameAttribute("Start AoA")]
        property Single startAoA;

        [CategoryAttribute("Controllers")]
        [DisplayNameAttribute("AoA params")]
        property List<Single> ^AoA_params;

        [CategoryAttribute("Controllers")]
        [DisplayNameAttribute("NN input lower")]
        property List<Single> ^InputLowerBounds;

        [CategoryAttribute("Controllers")]
        [DisplayNameAttribute("NN input upper")]
        property List<Single> ^InputUpperBounds;

        [CategoryAttribute("Controllers")]
        [DisplayNameAttribute("NN output lower")]
        property List<Single> ^OutputLowerBounds;

        [CategoryAttribute("Controllers")]
        [DisplayNameAttribute("NN output upper")]
        property List<Single> ^OutputUpperBounds;

        [Browsable(false)]
        property List<Single> ^outputVelHistory;

        virtual void execute() override;

    private:
        void randomize_params();
        void init_normals();
    };

}