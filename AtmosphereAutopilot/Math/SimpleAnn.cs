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

    /// <summary>
    /// Simple single hidden layer artificial neural network with tansig transfer function
    /// and purelin output transfer function.
    /// </summary>
    public class SimpleAnn
    {
        /// <summary>
        /// number of neurons in hidden layer
        /// </summary>
        public int hidden_count;

        /// <summary>
        /// number of network inputs
        /// </summary>
        public int input_count;

        double[] parameters;            // inearized weights and biases

        int fow;                        // index of first output layer weight
        int fb;                         // index of first bias

        /// <summary>
        /// Create new artificial neural network
        /// </summary>
        /// <param name="hidden_count">Number of neuronsin hidden layer</param>
        /// <param name="input_count">Number of network inputs</param>
        public SimpleAnn(int hidden_count, int input_count = 1)
        {
            this.hidden_count = hidden_count;
            this.input_count = input_count;
            fow = hidden_count * input_count;
            fb = hidden_count * (input_count + 1);
            parameters = new double[hidden_count * (input_count + 2) + 1];
            n1 = new double[hidden_count];
            a1 = new double[hidden_count];
            // randomise those weights and biases
            Random rand = new Random();
            for (int i = 0; i < parameters.Length; i++)
                parameters[i] = 1.0 * (rand.NextDouble() * 2.0 - 1.0);
        }

        /// <summary>
        /// Deep copy constructor
        /// </summary>
        public SimpleAnn(SimpleAnn original)
        {
            hidden_count = original.hidden_count;
            input_count = original.input_count;
            fow = original.fow;
            fb = original.fb;
            parameters = new double[hidden_count * (input_count + 2) + 1];
            n1 = new double[hidden_count];
            a1 = new double[hidden_count];
            // copy values of arrays
            original.parameters.CopyTo(parameters, 0);
            //original.n1.CopyTo(n1, 0);
            //original.a1.CopyTo(a1, 0);
        }

        /// <summary>
        /// Deep copy of ANN, creates exact copy
        /// </summary>
        public SimpleAnn Copy()
        {
            return new SimpleAnn(this);
        }

        /// <summary>
        /// First derivative of hyperbolic tangent
        /// </summary>
        public static double tanh_deriv(double x)
        {
            return 1.0 - Math.Pow(Math.Tanh(x), 2);
        }

        // net outputs of hidden layer neurons
        double[] n1;

        // scalar outputs of hidden layer neurons
        double[] a1;

        // Parameter indexers
        int hidden_weight(int neuron, int input)
        {
            return neuron * input_count + input;
        }

        int output_weight(int input_neuron)
        {
            return fow + input_neuron;
        }

        int hidden_bias(int neuron)
        {
            return fb + neuron;
        }

        int output_bias()
        {
            return fb + hidden_count;
        }

        /// <summary>
        /// Evaluate network on given inputs
        /// </summary>
        public double eval(Vector inputs)
        {
            return eval(inputs, parameters);
        }

        double eval(Vector inputs, double[] pars)
        {
            double a2 = 0.0;
            for (int n = 0; n < hidden_count; n++)
            {
                // propagate through hidden layer
                double n1 = 0.0;
                for (int i = 0; i < input_count; i++)
                    n1 += inputs[i] * pars[hidden_weight(n, i)];
                n1 += pars[hidden_bias(n)];
                double a1 = Math.Tanh(n1);
                // go to output layer
                a2 += pars[output_weight(n)] * a1;
            }
            a2 += pars[output_bias()];
            return a2;
        }

        double eval(Vector inputs, double[] pars, double[] n1, double[] a1)
        {
            double a2 = 0.0;
            for (int n = 0; n < hidden_count; n++)
            {
                // propagate through hidden layer
                n1[n] = 0.0;
                for (int i = 0; i < input_count; i++)
                    n1[n] += inputs[i] * pars[hidden_weight(n, i)];
                n1[n] += pars[hidden_bias(n)];
                a1[n] = Math.Tanh(n1[n]);
                // go to output layer
                a2 += pars[output_weight(n)] * a1[n];
            }
            a2 += pars[output_bias()];
            return a2;
        }


        #region Serialization

        public override string ToString()
        {
            string result =
                "hc=" + hidden_count.ToString() + ',' +
                "ic=" + input_count.ToString() + ',' +
                "params=" + string.Join(",", parameters.Select((x) => x.ToString("G8")).ToArray());
            return result;
        }

        /// <summary>
        /// Serialize network to ConfigNode
        /// </summary>
        /// <param name="node">In which node to put ann subnode</param>
        /// <param name="subnode_name">Name of ann subnode</param>
        public void SerializeToNode(ConfigNode node, string subnode_name)
        {
            ConfigNode ann_node = new ConfigNode(subnode_name);
            ann_node.AddValue("hidden_count", hidden_count.ToString());
            ann_node.AddValue("input_count", input_count.ToString());
            ann_node.AddValue("parameters", string.Join(",", parameters.Select((x) => x.ToString("G8")).ToArray()));
            node.AddNode(ann_node);
        }

        public static SimpleAnn DeserializeFromNode(ConfigNode node, string subnode_name)
        {
            ConfigNode ann_node = node.GetNode(subnode_name);
            if (ann_node == null)
                return null;
            try
            {
                int hidden_count = int.Parse(ann_node.GetValue("hidden_count"));
                int input_count = int.Parse(ann_node.GetValue("input_count"));
                double[] pars = ann_node.GetValue("parameters").Split(',').Select(s => double.Parse(s)).ToArray();
                if (pars.Length != hidden_count * (input_count + 2) + 1)
                    return null;
                return new SimpleAnn(hidden_count, input_count, pars);
            }
            catch (Exception)
            {
                return null;
            }
        }

        SimpleAnn(int hidden_count, int input_count, double[] pars)
        {
            this.hidden_count = hidden_count;
            this.input_count = input_count;
            fow = hidden_count * input_count;
            fb = hidden_count * (input_count + 1);
            this.parameters = pars;
            n1 = new double[hidden_count];
            a1 = new double[hidden_count];
        }

        #endregion


        #region Learning

        public class GaussKoeff
        {
            public double mu = 1e-3;
            double mu_min, mu_max;
            double tau_dec, tau_inc;

            public GaussKoeff(double mu_start, double min, double max, double dec, double inc)
            {
                mu = mu_start;
                mu_min = min;
                mu_max = max;
                tau_dec = dec;
                tau_inc = inc;
            }

            public void Success()
            {
                mu = Math.Max(mu / tau_dec, mu_min);
            }

            public void Fail()
            {
                mu = Math.Min(mu * tau_inc, mu_max);
            }
        }

        // Service containers for learning
        double[] new_params, new_errors;
        double[] errors;
        Matrix J, JtW, identity_storage,
            hessian, JtWe, JtWJ,
            temp_errors;

        public void preallocate(int expected_input_count)
        {
            int ic = expected_input_count + 1;
            int pl = parameters.Length;
            new_params = new double[pl];
            new_errors = new double[ic];
            errors = new double[ic];
            J = new Matrix(ic, pl);
            JtW = new Matrix(pl, ic);
            identity_storage = new Matrix(pl, pl);
            JtWJ = new Matrix(pl, pl);
            hessian = new Matrix(pl, pl);
            JtWe = new Matrix(pl, 1);
            temp_errors = new Matrix(ic, 1);
        }

        void lm_get_new_params(IList<double> pars, IList<double> new_pars, IList<double> errors, double mu)
        {
            // A
            Matrix.IdentityMatrix(pars.Count, pars.Count, ref identity_storage, mu);
            Matrix.Add(JtWJ, identity_storage, ref hessian);
            // b
            Matrix.ChangeData(errors, JtW.cols, ref temp_errors, true);
            Matrix.Multiply(JtW, temp_errors, ref JtWe);
            // get delta
            Matrix weight_delta = hessian.SolveWith(JtWe);
            for (int i = 0; i < pars.Count; i++)
                new_pars[i] = pars[i] - weight_delta[i, 0];
        }

        static double meansqr_tailed(double[] errors, IList<double> weights, double tail_weight, int error_count)
        {
            double res = 0;
            for (int i = 0; i < error_count - 1; i++)
            {
                double v = weights[i] * errors[i];
                res += v * v;
            }
            res += errors[error_count - 1] * tail_weight;
            return res / (double)error_count;
        }

        /// <summary>
        /// Perform complete cycle of Levenberg-Marquardt training algorithm. Weighted least squares with head batching is used as target function.
        /// </summary>
        /// <param name="inputs">Training set, vector of network inputs</param>
        /// <param name="outputs">Output for training set</param>
        /// <param name="error_weights">Weights for error_vector, use it to control importance of discrete input points</param>
        /// <param name="batch_size">From 0 to batch_size-1 inputs will be taken to form batch. Use when tou also need integral fitting.
        /// Use 0 when no batching is needed.</param>
        /// <param name="batch_weight">Importance of batch error</param>
        /// <param name="gauss">Gaussiness coeffitient</param>
        /// <param name="iter_limit">Descend iteration limit. Will break descend cycle if it's not possible</param>
        /// <param name="new_msqrerr">Get resulting performance index of the net</param>
        /// <returns>true if maanged to reduce error</returns>
        public bool lm_iterate_batched(IList<Vector> inputs, IList<double> outputs, IList<double> error_weights,
            int batch_size, double batch_weight, GaussKoeff gauss, int iter_limit, out double new_msqrerr)
        {
            int param_count = parameters.Length;
            int batch_count = batch_size > 0 ? 1 : 0;
            int error_count = inputs.Count + batch_count;

            // Allocate jacobian
            Matrix.Realloc(error_count, param_count, ref J);

            // Evaluate error vector and jacobian
            Common.Realloc(ref errors, error_count);
            for (int i = 0; i < inputs.Count; i++)                              // iterate over algorithm inputs
            {
                // errors
                double ann_output = eval(inputs[i], parameters, n1, a1);
                errors[i] = ann_output - outputs[i];
                // jacobian
                J[i, output_bias()] = 1.0;                                  // sensitivity of output neuron bias
                for (int n = 0; n < hidden_count; n++)                          // iterate over hidden neurons
                {
                    J[i, output_weight(n)] = a1[n];                         // sensitivity of output neuron weight, connected to n'th hidden neuron
                    double transf_deriv = tanh_deriv(n1[n]);                    // n'th hidden neuron transfer function deriv
                    double s1 = transf_deriv * parameters[output_weight(n)];    // sensitivity (of net output of n'th hidden neuron)
                    for (int p = 0; p < input_count; p++)                       // iterate over network inputs
                    {
                        J[i, hidden_weight(n, p)] = s1 * inputs[i][p];
                        J[i, hidden_bias(n)] = s1;
                    }
                }
            }

            // Evaluate batch error if needed
            if (batch_size > 0)
            {
                errors[error_count - 1] = 0.0;
                for (int par = 0; par < param_count; par++)
                    J[error_count - 1, par] = 0.0;
                for (int i = 0; i < batch_size; i++)
                {
                    for (int par = 0; par < param_count; par++)
                        J[error_count - 1, par] += J[i, par];       // summ jacobian rows to get batch row sensitivities
                    errors[error_count - 1] += errors[i];           // summ errors to get batch error
                }
            }

            // Calculate jacobian transpose and multiply it on jacobian itself
            Matrix.Transpose(J, ref JtW);
            // Multiply Jt on W (diagonal matrix of weights)
            for (int col = 0; col < JtW.cols - 1; col++)
                for (int row = 0; row < JtW.rows; row++)
                    JtW[row, col] = JtW[row, col] * error_weights[col];
            // Take batched row of jacobian into account
            for (int row = 0; row < JtW.rows; row++)
                JtW[row, error_count-1] = JtW[row, error_count-1] * batch_weight;
            Matrix.Multiply(JtW, J, ref JtWJ);

            // Evaluate meansqrerr if needed
            double old_msqrerr = meansqr_tailed(errors, error_weights, batch_weight, error_count);
            new_msqrerr = old_msqrerr;

            // Allocate storage for new params and errors if needed
            Common.Realloc(ref new_params, param_count);
            Common.Realloc(ref new_errors, error_count);

            // Try to perform Levenberg-Marquardt descend            
            int iter = 0;
            do
            {
                try
                {
                    lm_get_new_params(parameters, new_params, errors, gauss.mu);
                }
                catch (MSingularException)
                {
                    gauss.Fail();
                    iter++;
                    continue;
                }
                // Measure new meansqrerr
                for (int i = 0; i < inputs.Count; i++)
                {
                    double ann_output = eval(inputs[i], new_params);
                    new_errors[i] = (ann_output - outputs[i]) * error_weights[i];
                }
                if (batch_size > 0)
                {
                    new_errors[error_count - 1] = 0.0;
                    for (int i = 0; i < batch_size; i++)
                        new_errors[error_count - 1] += new_errors[i];
                    new_errors[error_count - 1] *= batch_weight;
                }
                new_msqrerr = meansqr_tailed(new_errors, error_weights, batch_weight, error_count);
                if (new_msqrerr < old_msqrerr)
                {
                    gauss.Success();
                    new_params = Interlocked.Exchange(ref parameters, new_params);
                    return true;
                }
                else
                    gauss.Fail();
                iter++;
            } while (new_msqrerr >= old_msqrerr && (iter < iter_limit));
            new_msqrerr = old_msqrerr;
            return false;
        }

        #endregion

    }

}
