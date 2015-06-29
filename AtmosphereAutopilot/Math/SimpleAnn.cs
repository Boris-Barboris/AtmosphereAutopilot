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
    /// and purelin output transfer function. It's learning and evaluation is thread-safe, 
    /// but learning must be performed from one thread only.
    /// </summary>
    public partial class SimpleAnn
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
                parameters[i] = rand.NextDouble() * 2.0 - 1;
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
            original.n1.CopyTo(n1, 0);
            original.a1.CopyTo(a1, 0);
            a2 = original.a2;
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

        // scalar output of output layer, wich is equal to it's net output
        double a2;

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
            a2 = 0.0;
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
                if (mu / tau_dec >= mu_min)
                    mu /= tau_dec;
            }

            public void Fail()
            {
                if (mu * tau_inc <= mu_max)
                    mu *= tau_inc;
            }
        }

        static void lm_get_weights(Matrix jacob, IList<double> pars, IList<double> new_pars, IList<double> errors, double mu)
        {
            Matrix jacob_trans = Matrix.Transpose(jacob);
            Matrix identity = Matrix.IdentityMatrix(pars.Count, pars.Count, mu);
            Matrix weight_delta = (jacob_trans * jacob + identity).Invert() * jacob_trans * new Matrix(errors, true);
            for (int i = 0; i < pars.Count; i++)
                new_pars[i] = pars[i] - weight_delta[i, 0];
        }

        // Service containers
        double[] new_params;

        /// <summary>
        /// Perform complete cycle of Levenberg-Marquardt training algorithm. Weighted least squares with head batching is used as target function.
        /// </summary>
        /// <param name="inputs">Training set, vector of network inputs</param>
        /// <param name="outputs">Output for training set</param>
        /// <param name="error_weights">Weights for error_vector, use it to control importance of discrete input points</param>
        /// <param name="batch_size">From 0 to batch_size-1 inputs will be taken to form batch. Use when need also integral fitting.
        /// Use 0 when no batching is needed.</param>
        /// <param name="batch_weight">Importance of batch error</param>
        /// <param name="gauss">Gaussiness coeffitient</param>
        /// <param name="iter_limit">Descend iteration limit. Will break descend cycle if it's not possible</param>
        /// <param name="new_msqrerr">Get resulting performance index of the net</param>
        /// <returns></returns>
        public bool lm_iterate_batched(IList<Vector> inputs, IList<double> outputs, IList<double> error_weights,
            int batch_size, double batch_weight, GaussKoeff gauss, int iter_limit, out double new_msqrerr)
        {
            int param_count = parameters.Length;
            int batch_count = batch_size > 0 ? 1 : 0;
            int error_count = inputs.Count + batch_count;

            // Allocate jacobian
            Matrix jacob = new Matrix(error_count, param_count);

            // Evaluate error vector and jacobian
            double[] weighted_errors = new double[error_count];
            for (int i = 0; i < inputs.Count; i++)                          // iterate over algorithm inputs
            {
                // errors
                double ann_output = eval(inputs[i]);
                weighted_errors[i] = (ann_output - outputs[i]) * error_weights[i];
                // jacobian
                jacob[i, output_bias()] = 1.0;                              // sensitivity of output neuron bias
                for (int n = 0; n < hidden_count; n++)                      // iterate over hidden neurons
                {
                    jacob[i, output_weight(n)] = a1[n];                     // output weight partial deriv
                    double transf_deriv = tanh_deriv(n1[n]);                // this neuron transfer function deriv
                    for (int p = 0; p < input_count; p++)                   // iterate over network inputs
                    {
                        double s1 = transf_deriv * parameters[output_weight(n)];       // marquardt sensitivity
                        jacob[i, hidden_weight(n, p)] = s1 * inputs[i][p];
                        jacob[i, hidden_bias(n)] = s1;
                    }
                }
            }

            // Evaluate batch error if needed
            if (batch_size > 0)
            {
                for (int i = 0; i < batch_size; i++)
                {
                    for (int par = 0; par < param_count; par++)
                        jacob[error_count - 1, par] += jacob[i, par];       // summ jacobian rows to get batch row
                    weighted_errors[error_count - 1] += weighted_errors[i]; // summ errors to get batch error
                }
                weighted_errors[error_count - 1] *= batch_weight;
            }

            // Evaluate meansqrerr if needed
            double old_msqrerr = weighted_errors.Meansqr();
            new_msqrerr = old_msqrerr;

            // Allocate storage for new params if needed
            if (new_params == null)
                new_params = new double[param_count];

            // Try to perform Levenberg-Marquardt descend
            double[] new_errors = new double[error_count];
            int iter = 0;
            do
            {
                try
                {
                    lm_get_weights(jacob, parameters, new_params, weighted_errors, gauss.mu);
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
                    for (int i = 0; i < batch_size; i++)
                        new_errors[error_count - 1] += new_errors[i];
                    new_errors[error_count - 1] *= batch_weight;
                }
                new_msqrerr = new_errors.Meansqr();
                if (new_msqrerr < old_msqrerr)
                {
                    gauss.Success();
                    double[] old_params = Interlocked.Exchange(ref parameters, new_params);
                    new_params = old_params;
                    return true;
                }
                else
                    gauss.Fail();
                iter++;
            } while (new_msqrerr >= old_msqrerr && (iter < iter_limit));

            return false;
        }
    }

}
