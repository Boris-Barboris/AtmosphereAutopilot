using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.IO;

namespace AtmosphereAutopilot
{
    /// <summary>
    /// PID with decreased frequency, for saw-like stiff functions like angular acceleration.
    /// </summary>
    class PIDAccumulating : PIDController
    {
        /// <summary>
        /// Controller will merge results with this frequency
        /// </summary>
        public int StepsToAccumulate 
        {
            get { return steps_to_sum; } 
            set 
            {
                if (steps_to_sum != value)
                {
                    steps_to_sum = value;
                    cur_step = 0;
                    error_buf = new CircularBuffer<double>(steps_to_sum, true);
                }
            } 
        }
        protected int steps_to_sum = 2;
        protected int cur_step = 0;
        
        internal double prev_output = 0.0;
        internal CircularBuffer<double> error_buf;

        public PIDAccumulating(int steps = 2)
        {
            steps_to_sum = steps;
            error_buf = new CircularBuffer<double>(steps_to_sum, true);
            last_error = double.NaN;
        }

        public override double Control(double input, double desire, double dt)
        {
            if (!dt_is_constant(dt))
            {
                // dt has changed
                error_buf.Clear();
                cur_step = 0;
            }
            last_dt = dt;

            double error = desire - input;

            // simply accumulate input
            error_buf.Put(error);

            cur_step++;
            if (cur_step < steps_to_sum)
                return prev_output;
            else
                cur_step = 0;
            
            // perform integral output calculation
            double avg_error = error_buf.Average();

			// proportional component
			double proportional = avg_error * kp;

			// update previous values   
			if (double.IsNaN(last_error))
				last_error = error_buf[0];
			derivative = (avg_error - last_error) / dt;
			double diffirential = derivative * kd;

            // integral component             
            double d_integral = 
                Math.Abs(avg_error) > iclamp ? 
                    0.0 :
                    steps_to_sum * dt * avg_error;         // raw diffirential
            d_integral = Common.Clamp(igain * d_integral, adclamp * dt * steps_to_sum);     // clamp it
            i_accumulator = Common.Clamp(i_accumulator + d_integral, aclamp);               // accumulate
            double integral = i_accumulator * ki;

            last_error = avg_error;

            prev_output = proportional + diffirential + integral;
			return prev_output;
        }

        public void clear_avg()
        {
            cur_step = 0;
            error_buf.Clear();
            last_error = double.NaN;
        }
    }
}
