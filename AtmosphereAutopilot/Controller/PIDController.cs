using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.ComponentModel;

namespace AtmosphereAutopilot
{

    /// <summary>
    /// PID-based input value controller
    /// </summary>
    class PIDController
    {
        /// <summary>
        /// Proportional gain coefficient
        /// </summary>
        public double KP { get { return kp; } set { kp = value; } }
        protected double kp = 1.0;

        /// <summary>
        /// Integral gain coefficient
        /// </summary>
        public double KI { get { return ki; } set { ki = value; } }
        protected double ki = 1.0;

        /// <summary>
        /// Diffirential gain coefficient
        /// </summary>
        public double KD { get { return kd; } set { kd = value; } }
        protected double kd = 0.01;

        /// <summary>
        /// Maximum error, wich lets integral component to raise
        /// </summary>
        public double IntegralClamp { get { return iclamp; } set { iclamp = value; } }
        protected double iclamp = 1.0;

        /// <summary>
        /// Maximum accumulator derivative
        /// </summary>
        public double AccumulDerivClamp { get { return adclamp; } set { adclamp = value; } }
        protected double adclamp = 1.0;

        /// <summary>
        /// Maximum magnitude for integral component of controller reaction
        /// </summary>
        public double AccumulatorClamp { get { return aclamp; } set { aclamp = value; } }
        protected double aclamp = 0.1;

        /// <summary>
        /// Diffirential gain, is multiplied on error * dt to get accumulator change
        /// </summary>
        public double IntegralGain { get { return igain; } set { igain = value; } }
        protected double igain = 1.0;

		public double InputDerivative { get { return derivative; } }

        public double Accumulator { get { return i_accumulator; } set { i_accumulator = value; } }

        // Main step variables
        protected double i_accumulator = 0.0;               // current integral accumulator state
        protected double last_dt = 1.0;                     // last delta time
        protected double last_error = 0.0;                  // last error
        protected double[] input_stack = new double[3];     // contains last 3 input values, needed for correct integration and differentiation
		protected double derivative;			// input derivatives

        public virtual double Control(double input, double desire, double dt)
        {
            double error = desire - input;
            double new_dt = dt;

            // proportional component
            double proportional = error * kp;
            
            // diffirential component
            if (!dt_is_constant(new_dt))
            {
                // dt has changed
                clean_value_stack(input);
                new_dt = TimeWarp.fixedDeltaTime;
                last_error = error;
            }
            update_value_stack(input);
            derivative = (input_stack[0] - 4 * input_stack[1] + 3 * input_stack[2]) / new_dt * 0.5;
            double diffirential = -derivative * kd;

            // integral component             
            double d_integral = Math.Abs(error) > iclamp ? 0.0 : new_dt * 0.5 * (error + last_error);       // raw diffirential
            d_integral = Common.Clamp(igain * d_integral, adclamp * new_dt);                                // clamp it
            i_accumulator = Common.Clamp(i_accumulator + d_integral, aclamp);                               // accumulate
            double integral = i_accumulator * ki;

            // update previous values
            last_dt = new_dt;
            last_error = error;

            return proportional + integral + diffirential;
        }

		public virtual double Control(double input, double input_d, double desire, double dt)
		{
			double error = desire - input;
			double new_dt = dt;

			// proportional component
			double proportional = error * kp;

			// diffirential component
			derivative = input_d;
			double diffirential = -input_d * kd;

			// integral component             
			double d_integral = Math.Abs(error) > iclamp ? 0.0 : new_dt * 0.5 * (error + last_error);       // raw diffirential
            d_integral = Common.Clamp(igain * d_integral, adclamp * new_dt);                                // clamp it
			i_accumulator = Common.Clamp(i_accumulator + d_integral, aclamp);                               // accumulate
			double integral = i_accumulator * ki;

			// update previous values
			last_dt = new_dt;
			last_error = error;

			return proportional + integral + diffirential;
		}

		/// <summary>
		/// Clear accumulator
		/// </summary>
        public void clear()
        {
            i_accumulator = 0.0;
        }

        protected void update_value_stack(double new_input)     // update value_stack with new input
        {
            input_stack[0] = input_stack[1];
            input_stack[1] = input_stack[2];
            input_stack[2] = new_input;
        }

        protected void clean_value_stack(double fill_value)      // need to clean value stack
        {
            for (int i = 0; i < 3; i++)
                input_stack[i] = fill_value;
        }

        protected bool dt_is_constant(double new_dt)            // return true if new dt is roughly the same as old one
        {
            if (Math.Abs(new_dt / last_dt - 1.0) < 0.1)
                return true;
            else
                return false;
        }

    }
}
