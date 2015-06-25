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

        static void lm_get_weights(Matrix jacob, IList<double> weights, IList<double> new_weights, IList<double> errors, double mu)
        {
            Matrix jacob_trans = Matrix.Transpose(jacob);
            Matrix identity = Matrix.IdentityMatrix(weights.Count, weights.Count, mu);
            Matrix weight_delta = (jacob_trans * jacob + identity).Invert() * jacob_trans * new Matrix(errors, true);
            for (int i = 0; i < weights.Count; i++)
                new_weights[i] = weights[i] - weight_delta[i, 0];
        }

        public bool lm_iterate_batched(IList<double[]> inputs, IList<double> error_weights,
            int batch_size, double batch_weight, double mu)
        {
            
        }
    }

}
