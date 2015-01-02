using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.ComponentModel;

namespace AtmosphereAutopilot
{
    /// <summary>
    /// Interface for all controller classes
    /// </summary>
    interface IController
    {
        /// <summary>
        /// Get controller reaction based on current input
        /// </summary>
        /// <param name="input">Current controlled value</param>
        /// <param name="desire">Desired controlled value</param>
        /// <param name="time">Current time</param>
        /// <returns>Controller reaction</returns>
        double Control(double input, double desire, double time);
    }

    /// <summary>
    /// PID-based input value controller
    /// </summary>
    class PIDController: IController
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
        protected double ki = 0.0;

        /// <summary>
        /// Diffirential gain coefficient
        /// </summary>
        public double KD { get { return kd; } set { kd = value; } }
        protected double kd = 0.0;

        /// <summary>
        /// Maximum magnitude for integral component of controller reaction
        /// </summary>
        public double IntegralClamp { get { return iclamp; } set { iclamp = value; } }
        protected double iclamp = 0.5;

        public double Accumulator { get { return i_accumulator; } set { i_accumulator = value; } }

        // Main step variables
        protected double i_accumulator = 0.0;               // current integral accumulator state
        protected double last_time = 0.0;                   // the time of last input
        protected double last_dt = 1.0;                     // last delta time
        protected double last_error = 0.0;                  // last delta time
        protected double[] input_stack = new double[3];     // contains last 3 input values, needed for correct integration and differentiation

        public virtual double Control(double input, double desire, double time)
        {
            double error = desire - input;
            // proportional component
            double proportional = error * kp;
            // integral component   
            double new_dt = time - last_time;
            if (new_dt == 0.0)
                return 0.0;
            double d_integral = new_dt * 0.5 * (error + last_error);
            i_accumulator = Common.Clamp(i_accumulator + d_integral, iclamp);
            double integral = i_accumulator * ki;
            // diffirential component
            if (!dt_is_constant(new_dt))
            {
                // dt has changed
                clean_value_stack(input);
                last_dt = new_dt;
                last_time = time - new_dt;
            }
            update_value_stack(input);
            double derivative = (input_stack[0] - 4 * input_stack[1] + 3 * input_stack[2]) / new_dt / new_dt;
            double diffirential = -derivative * kd;

            // update previous values
            last_time = time;
            last_dt = new_dt;
            last_error = error;

            return proportional + integral + diffirential;
        }

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
