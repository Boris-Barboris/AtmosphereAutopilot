#pragma once

using namespace System;
using namespace System::IO;
using namespace System::Collections::Generic;
using namespace System::ComponentModel;
using namespace System::ComponentModel::Design;

namespace AAGpuWrapper
{
    public ref class RawModelExperiment
    {
    public:
        RawModelExperiment()
        {
            this->dt = 0.05f;
            this->experiment_length = 5.0f;
            this->MOI = 165.0f;
        }

        [CategoryAttribute("Time parameters")]
        [DisplayNameAttribute("Time step, sec")]
        property Single dt;

        [CategoryAttribute("Time parameters")]
        [DisplayNameAttribute("Simulation length, sec")]
        property Single experiment_length;

        [CategoryAttribute("Craft parameters")]
        [DisplayNameAttribute("Pitch MOI")]
        property Single MOI;

        void execute();
    };

}