using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;

namespace AtmosphereAutopilot
{
    /// <summary>
    /// Flight control state controller base class
    /// </summary>
    public abstract class StateController : AutopilotModule
    {
		protected StateController(Vessel cur_vessel, string module_name, int wnd_id)
			: base(cur_vessel, wnd_id, module_name)
		{ }

		/// <summary>
		/// Main control function of high-level autopilot.
		/// </summary>
		/// <param name="cntrl">Control state to change</param>
		public abstract void ApplyControl(FlightCtrlState cntrl);
	}

	/// <summary>
	/// Flight control state controller with SIMO base class
	/// </summary>
	public abstract class SIMOController : AutopilotModule
	{
		protected SIMOController(Vessel cur_vessel, string module_name, int wnd_id)
			: base(cur_vessel, wnd_id, module_name)
		{ }

		[AutoGuiAttr("input", false, "G8")]
		public double input;					// current system controlled value

		[AutoGuiAttr("error", false, "G8")]
		public double error;					// desired - current

		[AutoGuiAttr("output", false, "G8")]
		public double output;					// current controller output

		/// <summary>
		/// Main control function of service autopilot.
		/// </summary>
		/// <param name="cntrl">Control state to change</param>
		/// <param name="target_value">Desired controlled value</param>
		public abstract double ApplyControl(FlightCtrlState cntrl, double target_value);

	}
}
