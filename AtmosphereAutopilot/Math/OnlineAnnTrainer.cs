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
        CircularBuffer<Vector> imm_buf_inputs;
        VectorArray imm_buf_vectors;
        CircularBuffer<double> imm_buf_outputs;        

        // Buffers for ANN training

        // Immediate buffer
        CircularBuffer<Vector> imm_training_inputs;
        VectorArray imm_training_vectors;
        CircularBuffer<double> imm_training_outputs;
        // Generalization buffer is being used as ANN generality augmentor
        GridSpace<GenStruct> gen_space;
        List<GridSpace<GenStruct>.CellValue> linear_gen_buff;
        List<Vector> gen_training_inputs = new List<Vector>();
        List<double> gen_training_outputs = new List<double>();        
        // Error weight buffers
        List<double> imm_error_weights = new List<double>();
        List<double> gen_error_weights = new List<double>();
        
        // Views to adapt inputs for ANN
        ListView<Vector> ann_input_view;
        ListView<double> ann_output_view;
        ListView<double> err_weight_view;

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
			linear_gen_buff = gen_space.Linearized;
            // Delegate assignment
            input_update_dlg = input_method;
            output_update_dlg = output_method;
            // Misc
            batch_size = imm_buf_size;
			temp_array = new VectorArray(ann.input_count, 1);
			coord_vector = temp_array[0];
        }

        /// <summary>
        /// While pushing new state to generalization space last CellBatch states will be averaged
        /// </summary>
        public volatile int cell_batch = 4;

        int last_time = 0;

        #region DataSourceThread

        public void UpdateState(int time_elapsed)
        {
            update_immediate_buffer();
            last_time += time_elapsed;
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

        SimpleAnn.GaussKoeff gauss = new SimpleAnn.GaussKoeff(1e-3, 1e-7, 1e6, 2.0, 100.0);

        public int batch_size;
        public volatile float batch_weight = 0.15f;
        public volatile float ann_performance = float.NaN;

        public void Train()
        {
            if (updated)
            {
                update_imm_train_buf();
                updated = false;
                update_gen_space();
                if (last_time > time_reset)
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
                        batch_weight, gauss, 3, out new_performance);
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

		VectorArray temp_array;
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
				for (int j = 0; j < dim; j++)
                    coord_vector[j] /= (double)cell_batch;
                gen_space.Put(new GenStruct(val, last_time), coord_vector);
            }
            if (linear_gen_buff.Count > 0)                      // let's update current state of generalization lists
            {
                for (int i = 0; i < linear_gen_buff.Count; i++)
                {
                    if (i >= gen_training_inputs.Count)
                        gen_training_inputs.Add(linear_gen_buff[i].coord);
                    else
                        gen_training_inputs[i] = linear_gen_buff[i].coord;
                    if (i >= gen_training_outputs.Count)
                        gen_training_outputs.Add(linear_gen_buff[i].data.val);
                    else
                        gen_training_outputs[i] = linear_gen_buff[i].data.val;
                }
                for (int i = gen_training_inputs.Count - 1; i >= linear_gen_buff.Count; i--)
                    gen_training_inputs.RemoveAt(i);
                for (int i = gen_training_outputs.Count - 1; i >= linear_gen_buff.Count; i--)
                    gen_training_outputs.RemoveAt(i);
            }
        }

        void create_views()
        {
            ann_input_view = new ListView<Vector>(imm_training_inputs, gen_training_inputs);
            ann_output_view = new ListView<double>(imm_training_outputs, gen_training_outputs);
            err_weight_view = new ListView<double>(imm_error_weights, gen_error_weights);
        }

        public volatile float base_gen_weight = 0.2f;

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

        public volatile float time_decay = 0.01f;
        public volatile int max_age = 100000;

        const int time_reset = int.MaxValue / 2;

        void reset_time()
        {
            foreach (var val in linear_gen_buff)
            {
                int cur_age = Math.Min(last_time - val.data.birth, max_age);
                val.data.birth = max_age - cur_age;
            }
            last_time = max_age;
        }

        double getAgeWeight(int birth)
        {
            int age = Math.Min(last_time - birth, max_age);
            double result = 1.0 / (time_decay * age + 1.0);
            return result;
        }

        #endregion

    }
}
