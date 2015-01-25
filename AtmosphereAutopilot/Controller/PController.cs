using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.ComponentModel;

namespace AtmosphereAutopilot
{

    /// <summary>
    /// Proportional controller
    /// </summary>
    public class PController
    {
        /// <summary>
        /// Proportional gain coefficient
        /// </summary>
        public double KP { get { return kp; } set { kp = value; } }
        protected double kp = 1.0;

		/// <summary>
		/// Last error value. Error = desire - input
		/// </summary>
		public double LastError { get { return last_error; } }
		protected double last_error = 0.0;

        public double Control(double input, double desire)
        {
			last_error = desire - input;

            // proportional component
			double proportional = last_error * kp;

            return proportional;
        }
    }
}
