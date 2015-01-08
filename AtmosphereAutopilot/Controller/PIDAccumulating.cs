using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace AtmosphereAutopilot
{
    /// <summary>
    /// PID with decreased frequency, for saw-like stiff functions like angular acceleration
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
                    input_buf = new double[steps_to_sum];
                }
            } 
        }
        protected int steps_to_sum = 4;
        protected int cur_step = 0;
        
        protected double prev_output_without_kp = 0.0;
        protected double[] input_buf;

        public PIDAccumulating(int steps = 4)
        {
            steps_to_sum = steps;
            input_buf = new double[steps];
        }

        public override double Control(double input, double desire, double dt)
        {
            if (!dt_is_constant(dt))
            {
                // dt has changed
                clean_value_stack(input);
                cur_step = 0;
            }
            last_dt = dt;

            // proportional component
            double proportional = (desire - input) * kp;

            // simply accumulate input
            input_buf[cur_step] = input;
            cur_step++;
            if (cur_step < steps_to_sum)
                return prev_output_without_kp + proportional;
            else
                cur_step = 0;
            
            // perform actual output calculation
            double avg_input = input_buf.Average();
            double error = desire - avg_input;

            update_value_stack(avg_input);
            derivative = (input_stack[0] - 4 * input_stack[1] + 3 * input_stack[2]) / steps_to_sum / dt * 0.5;
            double diffirential = -derivative * kd;

            // integral component             
            double d_integral = 
                Math.Abs(error) > iclamp ? 
                    0.0 :
                    steps_to_sum * dt * 0.5 * (error + last_error);             // raw diffirential
            d_integral = Common.Clamp(igain * d_integral, adclamp * dt * steps_to_sum);         // clamp it
            i_accumulator = Common.Clamp(i_accumulator + d_integral, aclamp);                   // accumulate
            double integral = i_accumulator * ki;

            // update previous values            
            last_error = error;

            prev_output_without_kp = integral + diffirential;
            return prev_output_without_kp + proportional;
        }
    }
}
