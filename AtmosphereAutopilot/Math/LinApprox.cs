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
using System.Threading;

namespace AtmosphereAutopilot
{

    using Vector = VectorArray.Vector;

    public class LinApprox
    {
        public readonly int input_count;

        public readonly double[] pars;                       // linear koeffitients

        public LinApprox(int input_count)
        {
            this.input_count = input_count;
            pars = new double[input_count + 1];
            tpars = new double[input_count + 1];
        }

        public double eval(Vector input)
        {
            lock (tpars)
            {
                double res = pars[0];
                for (int i = 0; i < input_count; i++)
                    res += pars[i + 1] * input[i];
                return res;
            }
        }

        public double eval_training(Vector input)
        {
            double res = tpars[0];
            for (int i = 0; i < input_count; i++)
                res += tpars[i + 1] * input[i];
            return res;
        }

        internal readonly double[] tpars;                   // training linear koefficients

        volatile bool updated_flag = false;                 // is set to true when training parameters are ready

        public void update_from_training()
        {
            if (updated_flag)
            {
                updated_flag = false;
                lock (tpars)
                {
                    tpars.CopyTo(pars, 0);
                }
            }
        }

        public void signalUpdated()
        {
            updated_flag = true;
        }
        
        Matrix X, Y, XtW, XtWX, XtWY;                       // matrix storages

        public void preallocate(int expected_input_count)
        {
            Matrix.Realloc(expected_input_count, input_count + 1, ref X);
            Matrix.Realloc(expected_input_count, 1, ref Y);
            Matrix.Realloc(input_count + 1, expected_input_count, ref XtW);
            Matrix.Realloc(input_count + 1, input_count + 1, ref XtWX);
            Matrix.Realloc(input_count + 1, input_count + 1, ref XtWY);
        }

        public bool weighted_lsqr(IList<Vector> inputs, IList<double> outputs, IList<double> weights, bool[] inputs_varied)
        {
            int icount = inputs.Count;
            int params_varied = 1;
            for (int i = 0; i < input_count; i++)
                if (inputs_varied[i])
                    params_varied++;
            
            // fill X and Y matrix
            Matrix.Realloc(icount, params_varied, ref X);
            Matrix.Realloc(icount, 1, ref Y);
            for (int i = 0; i < icount; i++)
            {
                X[i, 0] = 1.0;
                Y[i, 0] = outputs[i];
                int j = 0;
                for (int input = 0; input < input_count; input++)
                    if (inputs_varied[input])
                    {
                        j++;
                        X[i, j] = inputs[i][input];
                    }
                    else
                        Y[i, 0] -= inputs[i][input] * tpars[input + 1];
            }

            Matrix.Transpose(X, ref XtW);
            // Multiply Xt on W (diagonal matrix of weights)
            for (int col = 0; col < XtW.cols; col++)
                for (int row = 0; row < XtW.rows; row++)
                    XtW[row, col] = XtW[row, col] * weights[col];
            Matrix.Multiply(XtW, X, ref XtWX);
            Matrix.Multiply(XtW, Y, ref XtWY);
            try
            {
                Matrix new_params = XtWX.SolveWith(XtWY);
                lock (tpars)
                {
                    // update training parameters
                    tpars[0] = new_params[0, 0];
                    int j = 0;
                    for (int input = 0; input < input_count; input++)
                        if (inputs_varied[input])
                        {
                            j++;
                            tpars[input + 1] = new_params[j, 0];
                        }
                }
            }
            catch (MSingularException)
            {
                return false;
            }
            return true;
        }
    }

}
