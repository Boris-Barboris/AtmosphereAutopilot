/*
Copyright 2015, Boris-Barboris

This file is part of Atmosphere Autopilot.
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
        // Generalization buffer is being used as ANN generality augmentor
        GridSpace<GenStruct> gen_space;
        List<GridSpace<GenStruct>.CellValue> linear_gen_buff;
        // Error weight buffers
        List<double> imm_error_weights = new List<double>();
        List<double> gen_error_weights = new List<double>();
        
        // Views to adapt inputs for ANN
        ListView<Vector> input_view;
        ListView<double> output_view;
        ListView<double> weight_view;

        // flag of buffers update
        volatile bool updated = false;

        // time is used for generalization info ageing
        int cur_time = 0;

        // model to train
        LinApprox linmodel;
        readonly int input_count;

        struct GenStruct
        {
            public double val;
            public int birth;
            public GenStruct(double value, int time)
            {
                val = value;
                birth = time;
            }
        }

        public OnlineLinTrainer(LinApprox linprox, int imm_buf_size, int[] gen_cells, double[] l_gen_cell, double[] u_gen_cell,
            Action<Vector> input_method, Func<double> output_method)
        {
            this.linmodel = linprox;
            input_count = linmodel.input_count;
            // Immediate buffer initialization
            imm_buf_inputs = new CircularBuffer<Vector>(imm_buf_size, true);
            imm_buf_vectors = new VectorArray(input_count, imm_buf_size);
            imm_training_inputs = new CircularBuffer<Vector>(imm_buf_size, true);
            imm_training_vectors = new VectorArray(input_count, imm_buf_size);
            imm_buf_outputs = new CircularBuffer<double>(imm_buf_size, true);
            imm_training_outputs = new CircularBuffer<double>(imm_buf_size, true);
            // bind vectors in circular buffers to vector arrays
            for (int i = 0; i < imm_buf_size; i++)
            {
                imm_buf_inputs[i] = imm_buf_vectors[i];
                imm_training_inputs[i] = imm_training_vectors[i];
            }
            // Generalization space initialization
            init_lower_cell = l_gen_cell;
            init_upper_cell = u_gen_cell;
            double[] active_l_cell = new double[init_lower_cell.Length];
            double[] active_u_cell = new double[init_upper_cell.Length];
            init_lower_cell.CopyTo(active_l_cell, 0);
            init_upper_cell.CopyTo(active_u_cell, 0);
            gen_space = new GridSpace<GenStruct>(input_count, gen_cells, active_l_cell, active_u_cell);
			linear_gen_buff = gen_space.Linearized;
            gen_space.put_method = GenBufPutCriteria;
            // Delegates assignment
            input_update_dlg = input_method;
            output_update_dlg = output_method;
            // Preallocate buffers for matrix operations in linear approximator
            int supercell_size = gen_cells[0];
            for (int i = 1; i < input_count; i++)
                supercell_size *= gen_cells[i];
            linmodel.preallocate(imm_buf_size + supercell_size * 2 / 3);
            // Misc
            dist1 = new Vector(input_count);
            dist2 = new Vector(input_count);
            inputs_changed = new bool[input_count];
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

        public void Train()
        {
            if (updated)
            {
                update_imm_train_buf();
                updated = false;
                update_gen_space();
                if (cur_time > time_reset)
                    reset_time();
                update_weights();                
                if (input_view == null)
                    create_views();
                update_singularity();
                if (input_view != null)
                    if (input_view.Count > 0)
                    {
                        linmodel.weighted_lsqr(input_view, output_view, weight_view, inputs_changed);
                        check_linearity();
                    }
            }
        }

        [AutoGuiAttr("max_output_value", false)]
        double max_output_value = 0.01;         // maximum absolute value of model output reached in past

        readonly bool[] inputs_changed;         // flags that input has changed
        int added_to_imm = 0;                   // how many inputs where added to training imm_buf during last Train() call

        [AutoGuiAttr("max value decay", true)]
        public volatile float max_value_decay = 0.001f;

        void update_imm_train_buf()
        {
            // There are new inputs from Unity thread
            lock (imm_buf_inputs)
            {
                int count = imm_buf_inputs.Size;
                added_to_imm = count;
                while (count > 0)
                {
                    cur_time += last_time_elapsed;
                    Vector writing_cell = imm_training_inputs.getWritingCell();
                    imm_buf_inputs.Get().DeepCopy(writing_cell);
                    imm_training_inputs.Put(writing_cell);
                    double new_output = imm_buf_outputs.Get();
                    max_output_value = max_output_value * (1.0 - max_value_decay * last_time_elapsed);
                    max_output_value = Math.Max(Math.Max(max_output_value, Math.Abs(new_output)), 0.01);
                    imm_training_outputs.Put(new_output);
                    count--;
                }
            }
        }

        // initial values for generalization space bounds
        readonly double[] init_lower_cell;
        readonly double[] init_upper_cell;

        [AutoGuiAttr("gen region decay", true, "G8")]
        public volatile float gen_limits_decay = 0.0002f;     // how fast generalization space is shrinking by itself

        void update_gen_space()
        {
            // Shrink gen space
            if (gen_element_removed)
            {
                for (int j = 0; j < input_count; j++)
                {
                    gen_space.upper_cell[j] = Math.Max(init_upper_cell[j], linear_gen_buff.Max(v => v.coord[j]));
                    gen_space.lower_cell[j] = Math.Min(init_lower_cell[j], linear_gen_buff.Min(v => v.coord[j]));
                }
                gen_element_removed = false;
            }
            // Push all new inputs to generalization space
            for (int i = added_to_imm - 1; i >= 0; i--)
            {
                Vector new_coord = imm_training_inputs.getFromTail(i);
                double new_val = imm_training_outputs.getFromTail(i);
                // stretch generalization space size if needed
                for (int j = 0; j < input_count; j++)
                {
                    if (new_coord[j] > gen_space.upper_cell[j])
                        gen_space.upper_cell[j] = new_coord[j];
                    if (new_coord[j] < gen_space.lower_cell[j])
                        gen_space.lower_cell[j] = new_coord[j];
                }
                // decay
                //for (int j = 0; j < input_count; j++)
                //{
                //    double init_span = init_upper_cell[j] - init_lower_cell[j];
                //    double upper_span = gen_space.upper_cell[j] - new_coord[j];
                //    double decayed_upper_span = upper_span * (1.0 - last_time_elapsed * gen_limits_decay);
                //    gen_space.upper_cell[j] = new_coord[j] + Math.Max(decayed_upper_span, init_span / 2.0);
                //    double lower_span = gen_space.lower_cell[j] - new_coord[j];
                //    double decayed_lower_span = lower_span * (1.0 - last_time_elapsed * gen_limits_decay);
                //    gen_space.lower_cell[j] = new_coord[j] + Math.Min(decayed_lower_span, init_span / -2.0);
                //}
                gen_space.recompute_region();
                // push
                gen_space.Put(new GenStruct(new_val, cur_time - last_time_elapsed * i), new_coord);
            }            
        }

        void create_views()
        {
            input_view = new ListView<Vector>(imm_training_inputs, 
                new ListSelector<GridSpace<GenStruct>.CellValue, Vector>(linear_gen_buff, (cv) => { return cv.coord; }));
            output_view = new ListView<double>(imm_training_outputs,
                new ListSelector<GridSpace<GenStruct>.CellValue, double>(linear_gen_buff, (cv) => { return cv.data.val; }));
            weight_view = new ListView<double>(imm_error_weights, gen_error_weights);
        }

        [AutoGuiAttr("base gen weight", true, "G8")]
        public volatile float base_gen_weight = 0.1f;

        // Age decay factors for weights of inputs

        //[AutoGuiAttr("weight decay factor", false)]
        public volatile float weight_time_decay = 0.02f;

        [AutoGuiAttr("linear decay factor", true, "G8")]
        public volatile float linear_time_decay = 0.02f;

        [AutoGuiAttr("nonlinear decay factor", true, "G8")]
        public volatile float nonlin_time_decay = 0.5f;

        [AutoGuiAttr("min gen weight", false, "G8")]
        public volatile float min_gen_weight = 0.005f;

        [AutoGuiAttr("nonlin_cutoff_time", true)]
        public volatile int nonlin_cutoff_time  = 400;

        [AutoGuiAttr("linear criteria", true, "G8")]
        public volatile float linear_err_criteria = 0.05f;

        [AutoGuiAttr("linear", false)]
        public volatile bool linear = false;

        [AutoGuiAttr("linear_param", false, "G6")]
        public volatile float linear_param;

        void check_linearity()
        {
            if (imm_training_inputs.Size > 0)
            {
                //if (output_view != null)
                //    max_output_value = Math.Max(output_view.Max(v => Math.Abs(v)), 0.01);
                double sum_error = 0.0;
                for (int i = 0; i < imm_training_inputs.Size; i++)
                {
                    Vector input = imm_training_inputs[i];
                    double true_output = imm_training_outputs[i];
                    double lin_output = linmodel.eval_training(input);
                    double scaled_err = (lin_output - true_output) / max_output_value;
                    sum_error += Math.Abs(scaled_err);
                }
                linear_param = (float)(sum_error / (double)imm_training_inputs.Size);
                linear = (sum_error / (double)imm_training_inputs.Size) < linear_err_criteria;
                if (linear)
                    weight_time_decay = linear_time_decay;
                else
                    weight_time_decay = nonlin_time_decay;
            }
        }

        [AutoGuiAttr("gen_space_fill_k", false, "G6")]
        public volatile float gen_space_fill_k;

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
            // Generalization buffer weights
            gen_error_weights.Clear();
            int j = 0;
            min_gen_weight = (float)getAgeWeight(cur_time - nonlin_cutoff_time);
            while (j < linear_gen_buff.Count)
            {
                double decayed_weight = getAgeWeight(linear_gen_buff[j].data.birth);
                if (decayed_weight >= min_gen_weight || linear)
                {
                    double weight = decayed_weight * base_gen_weight;
                    gen_error_weights.Add(weight);
                    j++;
                }
                else
                {
                    // we need to delete this record from generalization space, it's too old
                    gen_space.Remove(linear_gen_buff[j]);
                    gen_element_removed = true;
                }
            }
            gen_space_fill_k = (float)(linear_gen_buff.Count / (double)gen_space.storage_length);
            double popul_ratio = (double)imm_training_inputs.Size / (double)linear_gen_buff.Count;
            for (int i = 0; i < gen_error_weights.Count; i++)
                gen_error_weights[i] *= popul_ratio;
        }

        void update_singularity()
        {
            for (int i = 0; i < input_count; i++)
                inputs_changed[i] = false;
            int changed = 0;
            for (int i = 0; i < input_view.Count - 1; i++)
            {
                if (changed >= input_count)
                    break;
                for (int j = 0; j < input_count; j++)
                {
                    if (inputs_changed[j])
                        continue;
                    if (input_view[i][j] != input_view[i + 1][j])
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
            for (int i = 0; i < linear_gen_buff.Count; i++)
            {
                int cur_age = Math.Min(cur_time - linear_gen_buff[i].data.birth, max_age);
                linear_gen_buff[i].data.birth = max_age - cur_age;
            }
            cur_time = max_age;
        }

        double getAgeWeight(int birth)
        {
            int age = Math.Min(cur_time - birth, max_age);
            double result = 1.0 / (weight_time_decay * age + 1.0);
            return result;
        }

        Vector dist1, dist2;

        void GenBufPutCriteria(GridSpace<GenStruct>.CellValue oldvalue, GenStruct newdata, Vector new_coord,
            Vector cell_center, double[] cell_sizes)
        {
            if (newdata.birth - oldvalue.data.birth > 100)
            {
                new_coord.DeepCopy(oldvalue.coord);
                oldvalue.data = newdata;
                return;
            }
            Vector.Sub(oldvalue.coord, cell_center, dist1);
            dist1.InverseScale(cell_sizes);
            Vector.Sub(new_coord, cell_center, dist2);
            dist2.InverseScale(cell_sizes);
            if (dist1.SqrLength() > dist2.SqrLength())
            {
                new_coord.DeepCopy(oldvalue.coord);
                oldvalue.data = newdata;
                return;
            }
        }

        #endregion

    }
}
