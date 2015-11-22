/*
Atmosphere Autopilot, plugin for Kerbal Space Program.
Copyright (C) 2015, Baranin Alexander aka Boris-Barboris.
 
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

    public class OnlineLinTrainer
    {
        // Buffers, that will be updated by calls from Unity event cycle

        // Immediate buffer
        CircularBuffer<Vector> imm_buf_inputs;
        VectorArray imm_buf_vectors;
        CircularBuffer<double> imm_buf_outputs;   

        // Buffers for linear regression

        // Immediate buffer
        CircularBuffer<Vector> imm_training_inputs;
        VectorArray imm_training_vectors;
        CircularBuffer<double> imm_training_outputs;
        // Gradient buffer
        VectorArray grad_training_vectors;
        CircularBuffer<GenStruct> grad_training;
        // Generalization buffer is being used as generality augmentor
		CircularBuffer<GenStruct>[] gen_buffers;
        // Error weight buffers
        List<double> imm_error_weights = new List<double>();
        List<double> grad_error_weights = new List<double>();
		List<double> gen_error_weights = new List<double>();
        
        // Views to adapt inputs for Approximator
        ListView<Vector> input_view;
        ListView<double> output_view;
        ListView<double> weight_view;

        // flag of buffers update
        volatile bool updated = false;

        // time is used for generalization measures aging
        int cur_time = 0;

        // model to train
        public readonly LinApprox linmodel, genmodel;
        readonly int input_count;

        struct GenStruct
        {
			public Vector coord;
            public double val;
            public int birth;
            public GenStruct(double value, int time, Vector v)
            {
                val = value;
                birth = time;
				coord = v;
            }
        }

        public OnlineLinTrainer(LinApprox linprox_main, LinApprox linprox_gen, int imm_buf_size, double[] gen_triggers, int[] gen_buf_sizes,
            Action<Vector> input_method, Func<double> output_method)
        {
            this.linmodel = linprox_main;
            this.genmodel = linprox_gen;
            input_count = linmodel.input_count;
            // Immediate and gradient buffer initialization
            imm_buf_inputs = new CircularBuffer<Vector>(imm_buf_size, true);
            imm_buf_vectors = new VectorArray(input_count, imm_buf_size);
            imm_training_inputs = new CircularBuffer<Vector>(imm_buf_size, true);
            imm_training_vectors = new VectorArray(input_count, imm_buf_size);
			grad_training = new CircularBuffer<GenStruct>(imm_buf_size / 2, true);
            grad_training_vectors = new VectorArray(input_count, imm_buf_size / 2);
            imm_buf_outputs = new CircularBuffer<double>(imm_buf_size, true);
            imm_training_outputs = new CircularBuffer<double>(imm_buf_size, true);
            // bind vectors in circular buffers to vector arrays
            for (int i = 0; i < imm_buf_size; i++)
            {
                imm_buf_inputs[i] = imm_buf_vectors[i];
                imm_training_inputs[i] = imm_training_vectors[i];
                if (i < imm_buf_size / 2)
                    grad_training[i] = new GenStruct(0.0, 0, grad_training_vectors[i]);
            }
            // Generalization space initialization
			this.gen_triggers = gen_triggers;
			gen_space_size = 0;
			gen_buffers = new CircularBuffer<GenStruct>[input_count];
			for (int i = 0; i < input_count; i++)
			{
				gen_buffers[i] = new CircularBuffer<GenStruct>(gen_buf_sizes[i], true);
				gen_space_size += gen_buf_sizes[i];
				VectorArray gen_array = new VectorArray(input_count, gen_buf_sizes[i]);
				for (int j = 0; j < gen_buf_sizes[i]; j++)
					gen_buffers[i][j] = new GenStruct(0.0, 0, gen_array[j]);
			}
            // Delegates assignment
            input_update_dlg = input_method;
            output_update_dlg = output_method;
            // Preallocate buffers for matrix operations in linear approximator
            linmodel.preallocate((int)(imm_buf_size * 1.5) + gen_buf_sizes.Sum());
            if (genmodel != null)
                genmodel.preallocate((int)(imm_buf_size * 1.5) + gen_buf_sizes.Sum());
            // Misc
            inputs_changed = new bool[input_count];
            nothing_changed = new bool[input_count];
        }

        #region DataSourceThread

        volatile int last_time_elapsed = 0;

        public void UpdateState(int time_elapsed)
        {
            update_immediate_buffer();
            last_time_elapsed = time_elapsed;
            updated = true;
        }

        Action<Vector> input_update_dlg;                // new state input getter
        Func<double> output_update_dlg;                 // new state output getter

        void update_immediate_buffer()
        {
            lock (imm_buf_inputs)
            {
                // inputs
                Vector input = imm_buf_inputs.getWritingCell();
                input_update_dlg(input);
                imm_buf_inputs.Put(input);
                // outputs
                imm_buf_outputs.Put(output_update_dlg());
            }
        }

        #endregion

        #region TrainingThread

        bool[] nothing_changed;

        public void Train()
        {
            if (updated)
            {
                update_train_buf();
                updated = false;
                if (cur_time > time_reset)
                    reset_time();
                update_weights();
                if (input_view == null)
                    create_views();
                if (input_view != null)
                {
                    if (input_view.Count > 0)
                    {
                        update_singularity(input_view);
                        linmodel.weighted_lsqr(input_view, output_view, weight_view, inputs_changed);
                        linmodel.weighted_lsqr(imm_training_inputs, imm_training_outputs, imm_error_weights, nothing_changed);
						linmodel.signalUpdated();
                    }
					if (genmodel != null && gen_list_view.Count > 0)
                    {
                        return_equal_weights();
                        genmodel.weighted_lsqr(input_view, output_view, weight_view, inputs_changed);
                        genmodel.weighted_lsqr(imm_training_inputs, imm_training_outputs, imm_error_weights, nothing_changed);
						genmodel.signalUpdated();
                    }
                    check_linearity();
                }
            }
        }

        [AutoGuiAttr("min_output_value", false, "G6")]
        public volatile float min_output_value = 0.01f;

        [AutoGuiAttr("max_output_value", false, "G6")]
        double max_output_value = 0.01;         // maximum absolute value of model output reached in past

        //[AutoGuiAttr("inputs_changed", false)]
        readonly bool[] inputs_changed;         // flags that input has changed

        int added_to_imm = 0;                   // how many inputs where added to training imm_buf during last Train() call

        [AutoGuiAttr("max value decay", true, "G6")]
        public volatile float max_value_decay = 0.001f;

        [AutoGuiAttr("gradient_sensitivity", true, "G6")]
        public volatile float gradient_sensitivity = 0.05f;

        void update_train_buf()
        {
            // There are new inputs from Unity thread
            lock (imm_buf_inputs)
            {
                int count = imm_buf_inputs.Size;
                added_to_imm = count;
                while (count > 0)
                {
                    cur_time += last_time_elapsed;
                    if (imm_training_inputs.Size == imm_training_inputs.Capacity)       // if we'll be overwriting immediate buffer
                    {
                        // let's update gradient buffers if needed
                        double cur_output = imm_training_outputs[1];
                        double prev_output = imm_training_outputs[0];
                        if (Math.Abs((prev_output - cur_output) / max_output_value) >= gradient_sensitivity)
                        {
                            // we need to put the output we'll be overwriting soon into gradient buffer
                            Vector wrt = grad_training.getWritingCell().coord;
                            Vector v2write = imm_training_inputs[0];
                            v2write.DeepCopy(wrt);
							grad_training.Put(new GenStruct(imm_training_outputs[0], cur_time - imm_training_inputs.Size * last_time_elapsed, wrt));
                        }
						// let's update generalization buffers
						Vector new_coord = imm_training_inputs[0];
						for (int j = 0; j < input_count; j++)
						{
							int cur_add_time = cur_time - imm_training_inputs.Size * last_time_elapsed;
							var prev_cell = gen_buffers[j].getLast();
							if (gen_buffers[j].Size == 0 ||
								Math.Abs(prev_cell.coord[j] - new_coord[j]) >
								(gen_buffers[j].Size / (double)gen_buffers[j].Capacity) * gen_triggers[j])
							{
								var cell = gen_buffers[j].getWritingCell();
								new_coord.DeepCopy(cell.coord);
								gen_buffers[j].Put(new GenStruct(prev_output, cur_add_time, cell.coord));
							}
						}
                    }
                    Vector writing_cell = imm_training_inputs.getWritingCell();
                    imm_buf_inputs.Get().DeepCopy(writing_cell);
                    imm_training_inputs.Put(writing_cell);
                    double new_output = imm_buf_outputs.Get();
                    max_output_value = max_output_value * (1.0 - max_value_decay * last_time_elapsed);
                    max_output_value = Math.Max(Math.Max(max_output_value, Math.Abs(new_output)), min_output_value);
                    imm_training_outputs.Put(new_output);
                    count--;
                }
            }
        }

		public double[] gen_triggers;

		ListView<GenStruct> gen_list_view;

        ListSelector<GenStruct, Vector> gen_inputs_selector;
        ListSelector<GenStruct, double> gen_output_selector;

		ListSelector<GenStruct, Vector> grad_input_selector;
		ListSelector<GenStruct, double> grad_output_selector;

        void create_views()
        {
			gen_list_view = new ListView<GenStruct>(gen_buffers);
			gen_inputs_selector = new ListSelector<GenStruct, Vector>(gen_list_view, cv => cv.coord);
			gen_output_selector = new ListSelector<GenStruct, double>(gen_list_view, cv => cv.val);
			grad_input_selector = new ListSelector<GenStruct, Vector>(grad_training, gs => gs.coord);
            grad_output_selector = new ListSelector<GenStruct, double>(grad_training, gs => gs.val);
			input_view = new ListView<Vector>(imm_training_inputs, grad_input_selector, gen_inputs_selector);
            output_view = new ListView<double>(imm_training_outputs, grad_output_selector, gen_output_selector);
            weight_view = new ListView<double>(imm_error_weights, grad_error_weights, gen_error_weights);
        }

        [AutoGuiAttr("base gen weight", true, "G8")]
        public volatile float base_gen_weight = 0.1f;

        // Age decay factors for weights of inputs

        //[AutoGuiAttr("weight decay factor", false)]
        public volatile float weight_time_decay = 0.02f;

        [AutoGuiAttr("linear decay factor", true, "G6")]
        public volatile float linear_time_decay = 0.02f;

        [AutoGuiAttr("nonlinear decay factor", true, "G6")]
        public volatile float nonlin_time_decay = 0.5f;

        [AutoGuiAttr("min gen weight", false, "G6")]
        public volatile float min_gen_weight = 0.005f;

        [AutoGuiAttr("nonlin_cutoff_time", true)]
        public volatile int nonlin_cutoff_time  = 1000;

        [AutoGuiAttr("linear criteria", true, "G6")]
        public volatile float linear_err_criteria = 0.05f;

        [AutoGuiAttr("linear", false)]
        public volatile bool linear = false;

        [AutoGuiAttr("linear_param", false, "G6")]
        public volatile float linear_param;

        [AutoGuiAttr("nonlin_cycles", false)]
        int nonlin_cycles = 0;

        void check_linearity()
        {
            if (imm_training_inputs.Size > 0)
            {
                double sum_error = 0.0;
                for (int i = 0; i < imm_training_inputs.Size; i++)
                {
                    Vector input = imm_training_inputs[i];
                    double true_output = imm_training_outputs[i];
                    double lin_output = (genmodel == null ? linmodel.eval_training(input) : genmodel.eval_training(input));
                    double scaled_err = (lin_output - true_output) / max_output_value;
                    sum_error += Math.Abs(scaled_err);
                }
                for (int i = 0; i < grad_training.Size; i++)
                {
                    Vector input = grad_training[i].coord;
                    double true_output = grad_training[i].val;
                    double lin_output = (genmodel == null ? linmodel.eval_training(input) : genmodel.eval_training(input));
                    double scaled_err = (lin_output - true_output) / max_output_value;
                    sum_error += Math.Abs(scaled_err);
                }
                //Vector input = imm_training_inputs.getLast();
                //double true_output = imm_training_outputs.getLast();
                //double lin_output = linmodel.eval_training(input);
                //sum_error = Math.Abs((lin_output - true_output) / max_output_value);
                linear_param = (float)(sum_error / (double)(imm_training_inputs.Size + grad_training.Size));
				linear = linear_param < linear_err_criteria;
                //linear_param = (float)sum_error;
                //linear = sum_error < linear_err_criteria;
                if (linear)
                {
                    weight_time_decay = linear_time_decay;
                    nonlin_cycles = 0;
                }
                else
                {
                    weight_time_decay = nonlin_time_decay;
                    nonlin_cycles += last_time_elapsed * added_to_imm;
                }
            }
        }

        [AutoGuiAttr("gen_space_fill_k", false, "G6")]
        public volatile float gen_space_fill_k;

		[AutoGuiAttr("nonlin_trigger", true)]
        public int nonlin_trigger = 100;

        [AutoGuiAttr("grad_buf_length", false)]
        int grad_buf_length { get { return grad_training.Size; } }

		int gen_space_size;

        bool gen_element_removed = false;

        void update_weights()
        {
            // Immediate buffer error weights
            imm_error_weights.Clear();
            for (int i = 0; i < imm_training_inputs.Size; i++)
            {
                int birth = cur_time - (imm_training_inputs.Size - i - 1) * last_time_elapsed;
                imm_error_weights.Add(getAgeWeight(birth));
            }
			// get cutoff decayed weight
			min_gen_weight = (float)getAgeWeight(cur_time - nonlin_cutoff_time);
            // Gradient buffer error weights
            grad_error_weights.Clear();
            int k = 0;
            while (k < grad_training.Size)
            {
                double k_weight = getAgeWeight(grad_training[k].birth);
                if (linear || k_weight >= min_gen_weight || nonlin_cycles < nonlin_trigger)
                {
                    grad_error_weights.Add(k_weight);
                    k++;
                }
                else
					grad_training.Get();
            }                
            // Generalization buffer weights
            gen_error_weights.Clear();
            for (int i = 0; i < gen_buffers.Length; i++)
				for (int j = 0; j < gen_buffers[i].Size; j++)
				{
					double decayed_weight = getAgeWeight(gen_buffers[i][j].birth);
                    if (linear || decayed_weight >= min_gen_weight || nonlin_cycles < nonlin_trigger || gen_buffers[i].Size <= 1)
					{
						double weight = decayed_weight * base_gen_weight;
						gen_error_weights.Add(weight);
					}
					else
					{
						// we need to delete this record from generalization space, it's too old
						gen_buffers[i].Get();
						gen_element_removed = true;
						j--;
					}
				}
			gen_space_fill_k = gen_error_weights.Count / (float)gen_space_size;
			double popul_ratio = (double)imm_training_inputs.Size / (double)gen_error_weights.Count;
            for (int i = 0; i < gen_error_weights.Count; i++)
                gen_error_weights[i] *= popul_ratio;
        }

        void return_equal_weights()
        {
			double popul_ratio = (double)imm_training_inputs.Size / (double)gen_error_weights.Count;
            for (int i = 0; i < gen_error_weights.Count; i++)
                gen_error_weights[i] /= popul_ratio * base_gen_weight;
        }

        void update_singularity(IList<Vector> input_list)
        {
            for (int i = 0; i < input_count; i++)
                inputs_changed[i] = false;
            int changed = 0;
            for (int i = 0; i < input_list.Count - 1; i++)
            {
                if (changed >= input_count)
                    break;
                for (int j = 0; j < input_count; j++)
                {
                    if (inputs_changed[j])
                        continue;
                    if (input_list[i][j] != input_list[i + 1][j])
                    {
                        inputs_changed[j] = true;
                        changed++;
                    }
                }
            }
        }

        const int time_reset = int.MaxValue / 2;

        [AutoGuiAttr("max age", true)]
        public volatile int max_age = 3000;

        void reset_time()
        {
            for (int i = 0; i < gen_buffers.Length; i++)
				for (int j = 0; j < gen_buffers[i].Size; j++)
				{
					var cur_struct = gen_buffers[i][j];
					int cur_age = Math.Min(cur_time - cur_struct.birth, max_age);
					gen_buffers[i][j] = new GenStruct(cur_struct.val, max_age - cur_age, cur_struct.coord);
				}            
            for (int i = 0; i < grad_training.Size; i++)
            {
                int cur_age = Math.Min(cur_time - grad_training[i].birth, max_age);
				grad_training[i] = new GenStruct(grad_training[i].val, max_age - cur_age, grad_training[i].coord);
            }
            cur_time = max_age;
        }

        double getAgeWeight(int birth)
        {
            int age = Math.Min(cur_time - birth, max_age);
            double result = 1.0 / (weight_time_decay * age + 1.0);
            return result;
        }

        #endregion

    }
}
