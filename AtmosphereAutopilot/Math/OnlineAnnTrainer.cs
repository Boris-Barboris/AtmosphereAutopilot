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

    public class OnlineAnnTrainer
    {
        // Buffers, that will be updated by calls from Unity event cycle

        // Immediate buffer
        CircularBufferAA<Vector> imm_buf_inputs;
        VectorArray imm_buf_vectors;
        CircularBufferAA<double> imm_buf_outputs;   

        // Buffers for ANN training

        // Immediate buffer
        CircularBufferAA<Vector> imm_training_inputs;
        VectorArray imm_training_vectors;
        CircularBufferAA<double> imm_training_outputs;
        // Generalization buffer is being used as ANN generality augmentor
        GridSpace<GenStruct> gen_space;
        List<GridSpace<GenStruct>.CellValue> linear_gen_buff;
        // Error weight buffers
        List<double> imm_error_weights = new List<double>();
        List<double> gen_error_weights = new List<double>();
        
        // Views to adapt inputs for ANN
        ListView<Vector> ann_input_view;
        ListView<double> ann_output_view;
        ListView<double> err_weight_view;

        // flag of buffers update
        volatile bool updated = false;

        /// <summary>
        /// While pushing new state to generalization space last CellBatch states will be averaged
        /// </summary>
        public volatile int cell_batch = 2;

        // time is used for generalization info aging
        int cur_time = 0;

        // ANN to train
        SimpleAnn ann;

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

        public OnlineAnnTrainer(SimpleAnn ann, int imm_buf_size, int[] gen_cells, double[] l_gen_bound, double[] u_gen_bound,
            Action<Vector> input_method, Func<double> output_method)
        {
            this.ann = ann;
            // Immediate buffer initialization
            imm_buf_inputs = new CircularBufferAA<Vector>(imm_buf_size, true);
            imm_buf_vectors = new VectorArray(ann.input_count, imm_buf_size);
            imm_training_inputs = new CircularBufferAA<Vector>(imm_buf_size, true);
            imm_training_vectors = new VectorArray(ann.input_count, imm_buf_size);
            imm_buf_outputs = new CircularBufferAA<double>(imm_buf_size, true);
            imm_training_outputs = new CircularBufferAA<double>(imm_buf_size, true);
            // bind vectors in circular buffers to vector arrays
            for (int i = 0; i < imm_buf_size; i++)
            {
                imm_buf_inputs[i] = imm_buf_vectors[i];
                imm_training_inputs[i] = imm_training_vectors[i];
            }
            // Generalization space initialization
            gen_space = new GridSpace<GenStruct>(ann.input_count, gen_cells, l_gen_bound, u_gen_bound);
            linear_gen_buff = gen_space.Linearized;
            gen_space.put_method = GenBufPutCriteria;
            // Delegates assignment
            input_update_dlg = input_method;
            output_update_dlg = output_method;
            // Preallocate buffers for ann
            int supercell_size = gen_cells[0];
            for (int i = 1; i < ann.input_count; i++)
                supercell_size *= gen_cells[i];
            ann.preallocate(imm_buf_size + supercell_size);
            // Misc
            batch_size = imm_buf_size;
            coord_vector = new Vector(ann.input_count);
        }

        #region DataSourceThread

        public void UpdateState(int time_elapsed)
        {
            update_immediate_buffer();
            cur_time += time_elapsed;
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

        SimpleAnn.GaussKoeff gauss = new SimpleAnn.GaussKoeff(1e-3, 1e-8, 1e8, 10.0, 100.0);

        [AutoGuiAttr("LM Mu", false, "G8")]
        public double Mu { get { return gauss.mu; } }

        [AutoGuiAttr("ann batch size", false)]
        public int batch_size;

        [AutoGuiAttr("ann batch weight", true)]
        public volatile float batch_weight = 0.0f;

        [AutoGuiAttr("ann performance", false, "G8")]
        public volatile float ann_performance = float.NaN;

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
                if (ann_input_view == null)
                    create_views();
            }
            if (ann_input_view != null)
                if (ann_input_view.Count > 0)
                {
                    double new_performance;
                    ann.lm_iterate_batched(ann_input_view, ann_output_view, err_weight_view, Math.Min(batch_size, imm_training_inputs.Size),
                        batch_weight, gauss, 4, out new_performance);
                    ann_performance = (float)new_performance;
                }
        }

        void update_imm_train_buf()
        {
            // There are new inputs from Unity thread
            lock (imm_buf_inputs)
            {
                int count = imm_buf_inputs.Size;
                while (count > 0)
                {
                    Vector writing_cell = imm_training_inputs.getWritingCell();
                    imm_buf_inputs.Get().DeepCopy(writing_cell);
                    imm_training_inputs.Put(writing_cell);
                    imm_training_outputs.Put(imm_buf_outputs.Get());
                    count--;
                }
            }
        }

        Vector coord_vector;

        void update_gen_space()
        {
            if (imm_training_inputs.Size >= cell_batch)         // there is enough material for generalization averaging
            {
                double val = 0.0;
                int dim = ann.input_count;
                for (int i = 0; i < dim; i++)
                    coord_vector[i] = 0.0;
                // Average state over cell_batch samples
                for (int i = 0; i < cell_batch; i++)
                {
                    val += imm_training_outputs.getFromTail(i);
                    for (int j = 0; j < dim; j++)
                        coord_vector[j] += imm_training_inputs.getFromTail(i)[j];
                }
                val /= (double)cell_batch;
                coord_vector.Scale(1.0/(double)cell_batch);
                gen_space.Put(new GenStruct(val, cur_time), coord_vector);
            }
        }

        void create_views()
        {
            ann_input_view = new ListView<Vector>(imm_training_inputs, 
                new ListSelector<GridSpace<GenStruct>.CellValue, Vector>(linear_gen_buff, (cv) => { return cv.coord; }));
            ann_output_view = new ListView<double>(imm_training_outputs,
                new ListSelector<GridSpace<GenStruct>.CellValue, double>(linear_gen_buff, (cv) => { return cv.data.val; }));
            err_weight_view = new ListView<double>(imm_error_weights, gen_error_weights);
        }

        [AutoGuiAttr("ann gener weight", true)]
        public volatile float base_gen_weight = 0.1f;

        void update_weights()
        {
            // Immediate buffer error weights
            if (imm_error_weights.Count < imm_training_inputs.Size)
            {
                int new_weights = imm_training_inputs.Size - imm_error_weights.Count;
                for (int i = 0; i < new_weights; i++)
                    imm_error_weights.Add(1.0);
            }
            else
                if (imm_error_weights.Count > imm_training_inputs.Size)
                {
                    int delete_weights = imm_error_weights.Count - imm_training_inputs.Size;
                    for (int i = 0; i < delete_weights; i++)
                        imm_error_weights.RemoveAt(imm_error_weights.Count - 1);
                }
            // Generalization buffer weights
            if (linear_gen_buff.Count > 0)
            {
                double popul_ratio = imm_training_inputs.Size / linear_gen_buff.Count;
                for (int i = 0; i < linear_gen_buff.Count; i++)
                {
                    double weight = getAgeWeight(linear_gen_buff[i].data.birth) * popul_ratio * base_gen_weight;
                    if (i >= gen_error_weights.Count)
                        gen_error_weights.Add(weight);
                    else
                        gen_error_weights[i] = weight;
                }
            }
        }

        [AutoGuiAttr("ann time decay", true)]
        public volatile float time_decay = 0.002f;

        [AutoGuiAttr("ann max_age", true)]
        public volatile int max_age = 100000;

        const int time_reset = int.MaxValue / 2;

        void reset_time()
        {
            foreach (var val in linear_gen_buff)
            {
                int cur_age = Math.Min(cur_time - val.data.birth, max_age);
                val.data.birth = max_age - cur_age;
            }
            cur_time = max_age;
        }

        double getAgeWeight(int birth)
        {
            int age = Math.Min(cur_time - birth, max_age);
            double result = 1.0 / (time_decay * age + 1.0);
            return result;
        }

        static void GenBufPutCriteria(GridSpace<GenStruct>.CellValue oldvalue, GenStruct newdata, Vector new_coord,
            Vector cell_center, double[] cell_sizes)
        {
            if (newdata.birth - oldvalue.data.birth > 200)
            {
                new_coord.DeepCopy(oldvalue.coord);
                oldvalue.data = newdata;
                return;
            }
            if (Vector.SqrLength(oldvalue.coord, cell_center) > Vector.SqrLength(new_coord, cell_center))
            {
                new_coord.DeepCopy(oldvalue.coord);
                oldvalue.data = newdata;
                return;
            }
        }

        #endregion

    }
}
