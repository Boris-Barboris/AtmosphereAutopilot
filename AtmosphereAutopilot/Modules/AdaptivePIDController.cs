using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;

namespace AtmosphereAutopilot
{
    /// <summary>
    /// PID-based auto-tuning controller
    /// </summary>
    abstract class AdaptivePIDController : AutopilotModule
    {
		public AdaptivePIDController(Vessel cur_vessel, string module_name, int wnd_id)
			: base(cur_vessel, wnd_id, module_name)
        {
            pid = new PIDController();
        }

        public AdaptivePIDController(Vessel cur_vessel, string module_name, int wnd_id, PIDController controller)
            : base(cur_vessel, wnd_id, module_name)
        {
            pid = controller;
        }

        protected PIDController pid;			// main PID controller

		[AutoGuiAttr("input", false, "G8")]
        public double input;					// current system controlled value

		[AutoGuiAttr("output", false, "G8")]
        public double output;				// current controller output

		[AutoGuiAttr("max_input", false, "G8")]
        public double max_input;			    // desired limitation on input value

		[AutoGuiAttr("min_input", false, "G8")]
        public double min_input;			    // desired limitation on input value

		[AutoGuiAttr("max_input_deriv", false, "G8")]
        public double max_input_deriv;		// desired limitation on input derivative value

		[AutoGuiAttr("min_input_deriv", false, "G8")]
        public double min_input_deriv;		// desired limitation on input derivative value

		/// <summary>
		/// Main control function
		/// </summary>
		/// <param name="cntrl">current control state</param>
		public abstract void ApplyControl(FlightCtrlState cntrl);

        /// <summary>
        /// Main control function
        /// </summary>
        /// <param name="cntrl">Control state to change</param>
        /// <param name="target_value">Desired angular acceleration</param>
        public abstract double ApplyControl(FlightCtrlState cntrl, double target_value);

        public static double apply_with_inertia(double old, double new_one, double inertia)
        {
            return (new_one + inertia * old) / (inertia + 1.0);
        }

		#region properties

        [AutoGuiAttr("Accumulator", false, "G8")]
        public double Accumulator { get { return pid.Accumulator; } }

		[VesselSerializable("KP")]
        [AutoGuiAttr("KP", false, "G8")]
		public double KP { get { return pid.KP; } set { pid.KP = value; } }

		[VesselSerializable("KI")]
        [AutoGuiAttr("KI", false, "G8")]
		public double KI { get { return pid.KI; } set { pid.KI = value; } }

		[VesselSerializable("KD")]
        [AutoGuiAttr("KD", false, "G8")]
		public double KD { get { return pid.KD; } set { pid.KD = value; } }

		[VesselSerializable("AccumulatorClamp")]
		[AutoGuiAttr("AccumulatorClamp", false, "G8")]
		public double AccumulatorClamp { get { return pid.AccumulatorClamp; } set { pid.AccumulatorClamp = value; } }

		[VesselSerializable("AccumulDerivClamp")]
		[AutoGuiAttr("AccumulDerivClamp", false, "G8")]
		public double AccumulDerivClamp { get { return pid.AccumulDerivClamp; } set { pid.AccumulDerivClamp = value; } }

		[VesselSerializable("IntegralClamp")]
		[AutoGuiAttr("IntegralClamp", false, "G8")]
		public double IntegralClamp { get { return pid.IntegralClamp; } set { pid.IntegralClamp = value; } }

        [VesselSerializable("IntegralGain")]
        [AutoGuiAttr("IntegralGain", false, "G8")]
        public double IntegralGain { get { return pid.IntegralGain; } set { pid.IntegralGain = value; } }

		#endregion
	}
}
