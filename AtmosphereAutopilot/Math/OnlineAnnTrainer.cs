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
        // Immediate buffer contains recent state records
        CircularBuffer<Vector> imm_buf_inputs;
        VectorArray imm_buf_vectors;
        CircularBuffer<double> imm_buf_outputs;

        CircularBuffer<Vector> imm_training_inputs;         // immediate buffer, used directly for training
        VectorArray imm_training_vectors;
        CircularBuffer<double> imm_training_outputs;

        // Generalization buffer is being used as ANN generality augmentor
        GridSpace<GenStruct> gen_space;

        // flag of buffers update
        volatile bool updated = false;

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
            imm_buf_inputs = new CircularBuffer<Vector>(imm_buf_size, true);
            imm_buf_vectors = new VectorArray(ann.input_count, imm_buf_size);
            imm_training_inputs = new CircularBuffer<Vector>(imm_buf_size, true);
            imm_training_vectors = new VectorArray(ann.input_count, imm_buf_size);
            imm_buf_outputs = new CircularBuffer<double>(imm_buf_size, true);
            imm_training_outputs = new CircularBuffer<double>(imm_buf_size, true);
            // bind vectors in circular buffers to vector arrays
            for (int i = 0; i < imm_buf_size; i++)
            {
                imm_buf_inputs[i] = imm_buf_vectors[i];
                imm_training_inputs[i] = imm_training_vectors[i];
            }
            // Generalization space initialization
            gen_space = new GridSpace<GenStruct>(ann.input_count, gen_cells, l_gen_bound, u_gen_bound);
            // Delegates assignment
            input_update_dlg = input_method;
            output_update_dlg = output_method;
            // Misc
            batch_size = imm_buf_size;
        }

        /// <summary>
        /// While pushing new state to generalization space last CellBatch states will be averaged
        /// </summary>
        public int cell_batch = 4;

        int last_time = 0;

        #region DataSourceThread

        public void UpdateState(int time)
        {
            update_immediate_buffer();
            last_time = time;
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

        List<double> imm_error_weights = new List<double>();
        List<double> gen_error_weights = new List<double>();

        ListView<Vector> ann_input_view;
        ListView<double> ann_output_view;
        ListView<double> err_weight_view;

        SimpleAnn.GaussKoeff gauss = new SimpleAnn.GaussKoeff(1e-3, 1e-7, 1e6, 2.0, 100.0);

        public int batch_size;
        public double batch_weight = 0.15;
        public double ann_performance = double.NaN;

        public void Train()
        {
            if (updated)
            {
                update_imm_train_buf();
                updated = false;
                update_gen_space();
                update_weights();
                update_views();           
            }
            if (ann_input_view != null)
                ann.lm_iterate_batched(ann_input_view, ann_output_view, err_weight_view, Math.Min(batch_size, imm_training_inputs.Size),
                    batch_weight, gauss, 3, out ann_performance);
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

        int last_gen_index = -2;

        List<GridSpace<GenStruct>.CellValue> linear_gen_buff = new List<GridSpace<GenStruct>.CellValue>();

        void update_gen_space()
        {
            if (imm_training_inputs.Size >= cell_batch)         // there is enough material for generalization averaging
            {
                double[] gen_coord = new double[ann.input_count];
                double val = 0.0;
                for (int i = 0; i < ann.input_count; i++)
                    gen_coord[i] = 0.0;
                // Average state over cell_batch samples
                for (int i = 0; i < cell_batch; i++)
                {
                    val += imm_buf_outputs.getFromTail(i);
                    double[] inputs = imm_buf_inputs.getFromTail(i);
                    for (int j = 0; j < inputs.Length; j++)
                        gen_coord[j] += inputs[j];
                }
                val /= (double)cell_batch;
                for (int j = 0; j < gen_coord.Length; j++)
                    gen_coord[j] /= (double)cell_batch;
                // check if we switched to new cell
                int cellid = gen_space.getCellIdForCoord(gen_coord);
                if (cellid != last_gen_index)
                {
                    // Push state to generalization space
                    gen_space.Put(new GenStruct(val, last_time), gen_coord);
                    last_gen_index = cellid;
                    // We need to update generalization buffer linear representation
                    linear_gen_buff = gen_space.linearize();
                }
            }
        }

        void update_views()
        {
            ann_input_view = new ListView<double[]>(imm_training_inputs, linear_gen_buff.Select(v => v.coord).ToArray());
            ann_output_view = new ListView<double>(imm_training_outputs, linear_gen_buff.Select(v => v.data.val).ToArray());
            err_weight_view = new ListView<double>(imm_error_weights, gen_error_weights);
        }

        public double base_gen_weight = 0.2;

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
                gen_error_weights = new List<double>(linear_gen_buff.Select(v => getAgeWeight(v.data.birth) * popul_ratio * base_gen_weight));
            }
        }

        public double time_decay = 1.0;

        double getAgeWeight(int birth)
        {
            int age = last_time - birth;
            double result = 1.0 / (time_decay * age + 1.0);
            return result;
        }

        #endregion

    }
}
