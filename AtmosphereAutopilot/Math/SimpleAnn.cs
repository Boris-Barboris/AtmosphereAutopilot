using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace AtmosphereAutopilot
{

    /// <summary>
    /// Simple single hidden layer artificial neural network with tansig transfer function
    /// and purelin output transfer function.
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

        double[] weights;               // linearized neuron weights
        double[] biases;                // linearized network biases

        int fow;                        // index of first output layer weight

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
            weights = new double[hidden_count * (input_count + 1)];
            biases = new double[hidden_count + 1];
            n1 = new double[hidden_count];
            a1 = new double[hidden_count];
            // randomise those weights and biases
            Random rand = new Random();
            for (int i = 0; i < weights.Length; i++)
                weights[i] = rand.NextDouble() * 2.0 - 1;
            for (int i = 0; i < biases.Length; i++)
                biases[i] = rand.NextDouble() * 2.0 - 1;
        }

        /// <summary>
        /// Deep copy constructor
        /// </summary>
        public SimpleAnn(SimpleAnn original)
        {
            hidden_count = original.hidden_count;
            input_count = original.input_count;
            fow = original.fow;
            weights = new double[hidden_count * (input_count + 1)];
            biases = new double[hidden_count + 1];
            n1 = new double[hidden_count];
            a1 = new double[hidden_count];
            // copy values of arrays
            original.weights.CopyTo(weights, 0);
            original.biases.CopyTo(biases, 0);
            original.n1.CopyTo(n1, 0);
            original.a1.CopyTo(a1, 0);
            a2 = original.a2;
        }

        /// <summary>
        /// Deep copy of object
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

        /// <summary>
        /// net outputs of hidden layer neurons
        /// </summary>
        public double[] n1;

        /// <summary>
        /// scalar outputs of hidden layer neurons
        /// </summary>
        public double[] a1;

        /// <summary>
        /// scalar output of output layer, wich is equal to it's net output
        /// </summary>
        public double a2;

        /// <summary>
        /// Evaluate network on given inputs
        /// </summary>
        public double eval(params double[] inputs)
        {
            a2 = 0.0;
            for (int n = 0; n < hidden_count; n++)
            {
                // propagate through hidden layer
                n1[n] = 0.0;
                for (int i = 0; i < input_count; i++)
                    n1[n] += inputs[i] * weights[n * input_count + i];
                n1[n] += biases[n];
                a1[n] = Math.Tanh(n1[n]);
                // go to output layer
                a2 += weights[fow + n] * a1[n];
            }
            a2 += biases[hidden_count];
            return a2;
        }

        #region Serialization

        public override string ToString()
        {
            string result =
                "hc=" + hidden_count.ToString() + ',' +
                "ic=" + input_count.ToString() + ',' +
                "w=" + string.Join(",", weights.Select((x) => x.ToString("G8")).ToArray()) + ',' +
                "b=" + string.Join(",", biases.Select((x) => x.ToString("G8")).ToArray());
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
            ann_node.AddValue("weights", string.Join(",", weights.Select((x) => x.ToString("G8")).ToArray()));
            ann_node.AddValue("biases", string.Join(",", biases.Select((x) => x.ToString("G8")).ToArray()));
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
                double[] weights = ann_node.GetValue("weights").Split(',').Select(s => double.Parse(s)).ToArray();
                if (weights.Length != hidden_count * (input_count + 1))
                    return null;
                double[] biases = ann_node.GetValue("biases").Split(',').Select(s => double.Parse(s)).ToArray();
                if (biases.Length != hidden_count + 1)
                    return null;
                return new SimpleAnn(hidden_count, input_count, weights, biases);
            }
            catch (Exception)
            {
                return null;
            }
        }

        SimpleAnn(int hidden_count, int input_count, double[] weights, double[] biases)
        {
            this.hidden_count = hidden_count;
            this.input_count = input_count;
            fow = hidden_count * input_count;
            this.weights = weights;
            this.biases = biases;
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

        // Jacobian indexers
        int hidden_weight(int neuron, int input)
        {
            return neuron * input_count + input;
        }

        int output_weight(int neuron_tail)
        {
            return fow + neuron_tail;
        }

        int hidden_bias(int neuron)
        {
            return weights.Length + neuron;
        }

        // Service containers
        double[] new_weights;
        double[] new_biases;
        ListView<double> params_view;
        ListView<double> new_params_view;

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
        public bool lm_iterate_batched(IList<double[]> inputs, IList<double> outputs, IList<double> error_weights,
            int batch_size, double batch_weight, GaussKoeff gauss, int iter_limit, out double new_msqrerr)
        {
            int param_count = weights.Length + biases.Length;            
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
                jacob[i, param_count - 1] = 1.0;                            // sensitivity of output neuron bias
                for (int n = 0; n < hidden_count; n++)                      // iterate over hidden neurons
                {
                    jacob[i, output_weight(n)] = a1[n];                     // output weight partial deriv
                    double transf_deriv = tanh_deriv(n1[n]);                // this neuron transfer function deriv
                    for (int p = 0; p < input_count; p++)                   // iterate over network inputs
                    {
                        double s1 = transf_deriv * weights[output_weight(n)];       // marquardt sensitivity
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

            // Allocate storage for new weights if needed
            if (new_weights == null)
                new_weights = new double[hidden_count * (input_count + 1)];
            if (new_biases == null)
                new_biases = new double[hidden_count + 1];
            // Prepare parameter views if needed
            if (params_view == null)
                params_view = new ListView<double>(weights, biases);
            if (new_params_view == null)
                new_params_view = new ListView<double>(new_weights, new_biases);

            // Try to perform Levenberg-Marquardt descend
            double[] new_errors = new double[error_count];
            int iter = 0;
            do
            {
                try
                {
                    lm_get_weights(jacob, params_view, new_params_view, weighted_errors, gauss.mu);
                }
                catch (MSingularException)
                {
                    gauss.Fail();
                    iter++;
                    continue;
                }
                // Measure new meansqrerr
                double[] saved_weights = weights;
                double[] saved_biases = biases;
                weights = new_weights;
                biases = new_biases;
                for (int i = 0; i < inputs.Count; i++)
                {
                    double ann_output = eval(inputs[i]);
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
                    gauss.Success();
                else
                {
                    gauss.Fail();
                    weights = saved_weights;
                    biases = saved_biases;
                }
                iter++;
            } while (new_msqrerr >= old_msqrerr && (iter < iter_limit));

            return (new_msqrerr < old_msqrerr);
        }
    }

}
