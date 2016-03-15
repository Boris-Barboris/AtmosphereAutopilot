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
using System.ComponentModel;

namespace AtmosphereAutopilot
{

    /// <summary>
    /// PD-based input value controller
    /// </summary>
    public class PDController
    {
        /// <summary>
        /// Proportional gain coefficient
        /// </summary>
        public double KP { get { return kp; } set { kp = value; } }
        protected double kp = 1.0;

        /// <summary>
        /// Diffirential gain coefficient
        /// </summary>
        public double KD { get { return kd; } set { kd = value; } }
        protected double kd = 0.01;

		/// <summary>
		/// Input derivative value from the last control iteration
		/// </summary>
		public double InputDerivative { get { return derivative; } }
        protected double derivative;			            // input derivative value

        // Main step variables
        protected double last_dt = 1.0;                     // last delta time

		/// <summary>
		/// Last error value. Error = desire - input
		/// </summary>
        public double LastInput { get { return last_input; } }
		protected double last_input = 0.0;	

        public double Control(double input, double desire, double dt)
        {
            double error = desire - input;
            double new_dt = dt;

            // proportional component
            double proportional = error * kp;

            // diffirential component
            if (!dt_is_constant(new_dt))
            {
                // dt has changed
                new_dt = TimeWarp.fixedDeltaTime;
                last_input = input;
            }
            if (kd != 0.0)
                derivative = (input - last_input) / new_dt;
            double diffirential = -derivative * kd;

            // update previous values
            last_dt = new_dt;
            last_input = error;

            return proportional + diffirential;
        }

        /// <summary>
        /// Control function with manually provided input derivative
        /// </summary>
		public double Control(double input, double input_d, double desire, double dt)
		{
            double error = desire - input;
            double new_dt = dt;

            // proportional component
            double proportional = error * kp;

            // diffirential component
            if (!dt_is_constant(new_dt))
            {
                // dt has changed
                new_dt = TimeWarp.fixedDeltaTime;
                last_input = input;
            }
            derivative = input_d;
            double diffirential = -derivative * kd;

            // update previous values
            last_dt = new_dt;
            last_input = input;

            return proportional + diffirential;
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
