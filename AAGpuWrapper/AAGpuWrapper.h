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

    public ref class RawModelExperiment
    {
    public:
        RawModelExperiment()
        {
            dt = 0.05f;
            experiment_length = 10.0f;
            MOI = 165.0f;
            mass = 14.0f;
            sas = 15.0;
            pitchRotModel = gcnew array<Single> { 0.0f, -1.0f, 1.15f };
            pitchLiftModel = gcnew array<Single> { 0.0f, 60.0f, -0.25f };
            dragModel = gcnew array<Single> { 1.0f, 20.0f };
            aerodynamics = AeroModel::StockAero;
            startVel = 200.0f;
            keepSpeed = false;
            control = 0.0f;
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
        property array<Single> ^pitchRotModel;

        [CategoryAttribute("Craft parameters")]
        [DisplayNameAttribute("Pitch lift model")]
        property array<Single> ^pitchLiftModel;

        [CategoryAttribute("Craft parameters")]
        [DisplayNameAttribute("Drag model")]
        property array<Single> ^dragModel;

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

}