using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.IO;

namespace AtmosphereAutopilot
{
    /// <summary>
    /// PID with decreased frequency, for saw-like stiff functions like angular acceleration. Recalculates accumulator more seldom
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
        protected int steps_to_sum = 4;
        protected int cur_step = 0;
        
        protected double prev_integral = 0.0;
        internal CircularBuffer<double> error_buf;

        public double AverageInputDerivative { get { return avg_derivative; } }
        protected double avg_derivative;			            // input derivative value

        public PIDAccumulating(int steps = 4)
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
            // proportional component
            double proportional = error * kp;

            // simply accumulate input
            error_buf.Put(error);

            // error derivative component
            if (error_buf.Size <= 1)
                error_buf.Put(error);
            derivative = (error - error_buf.getFromTail(1)) / dt;
            double diffirential = derivative * kd;

            cur_step++;
            if (cur_step < steps_to_sum)
                return prev_integral + proportional + diffirential;
            else
                cur_step = 0;
            
            // perform integral output calculation
            double avg_error = error_buf.Average();

            // integral component             
            double d_integral = 
                Math.Abs(avg_error) > iclamp ? 
                    0.0 :
                    steps_to_sum * dt * avg_error;         // raw diffirential
            d_integral = Common.Clamp(igain * d_integral, adclamp * dt * steps_to_sum);     // clamp it
            i_accumulator = Common.Clamp(i_accumulator + d_integral, aclamp);               // accumulate
            double integral = i_accumulator * ki;

            // update previous values   
            if (double.IsNaN(last_error))
                last_error = error_buf[0];
            avg_derivative = (avg_error - last_error) / dt / steps_to_sum;
            last_error = avg_error;

            prev_integral = integral;
            return prev_integral + proportional + diffirential;
        }

        public void clear_avg()
        {
            cur_step = 0;
            error_buf.Clear();
            last_error = double.NaN;
        }
    }
}
