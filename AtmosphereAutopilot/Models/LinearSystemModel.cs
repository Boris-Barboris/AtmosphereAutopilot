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
    }
}
