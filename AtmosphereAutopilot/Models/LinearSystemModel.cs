/*
Atmosphere Autopilot, plugin for Kerbal Space Program.
Copyright (C) 2015-2016, Baranin Alexander aka Boris-Barboris.
 
Atmosphere Autopilot is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
Atmosphere Autopilot is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
You should have received a copy of the GNU General Public License
along with Atmosphere Autopilot.  If not, see <http://www.gnu.org/licenses/>. 
*/

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;

namespace AtmosphereAutopilot
{
    public struct LinearSystemModel
    {
        public readonly int state_count, input_count;

        public readonly Matrix A, B, C;

        public LinearSystemModel(int state_count, int input_count)
        {
            this.state_count = state_count;
            this.input_count = input_count;
            A = new Matrix(state_count, state_count);
            B = new Matrix(state_count, input_count);
            C = new Matrix(state_count, 1);
            Ax = Bu = AxBu = null;
        }

        public LinearSystemModel(LinearSystemModel original)
        {
            state_count = original.state_count;
            input_count = original.input_count;
            A = original.A.Duplicate();
            B = original.B.Duplicate();
            C = original.C.Duplicate();
            Ax = Bu = AxBu = null;
        }

        Matrix Ax;
        Matrix Bu;
        Matrix AxBu;

        public void eval(Matrix state, Matrix input, ref Matrix state_deriv)
        {
            Matrix.Multiply(A, state, ref Ax);
            Matrix.Multiply(B, input, ref Bu);
            Matrix.Add(Ax, Bu, ref AxBu);
            Matrix.Add(AxBu, C, ref state_deriv);
        }

        public double eval_row(int row, Matrix state, Matrix input)
        {
            double res = 0.0;
            for (int i = 0; i < A.cols; i++)
                res += A[row, i] * state[i, 0];
            for (int i = 0; i < B.cols; i++)
                res += B[row, i] * input[i, 0];
            res += C[row, 0];
            return res;
        }
    }
}
