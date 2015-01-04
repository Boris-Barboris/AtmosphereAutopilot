using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;

namespace AtmosphereAutopilot
{
    /// <summary>
    /// Any autopilot component. For example, roll damper
    /// </summary>
    abstract class PIDAutoTrimmer : AutopilotModule
    {
		public PIDAutoTrimmer(Vessel cur_vessel, string module_name, int wnd_id)
			: base(cur_vessel, wnd_id, module_name)
        {
            pid = new PIDController();
        }

        protected PIDController pid;
        protected double angular_velocity;
        protected double output;

		protected override void OnActivate()
        {
            vessel.OnAutopilotUpdate += new FlightInputCallback(OnFixedUpdate);
        }

		protected override void OnDeactivate()
        {
            vessel.OnAutopilotUpdate -= new FlightInputCallback(OnFixedUpdate);
        }

        protected abstract void OnFixedUpdate(FlightCtrlState cntrl);



		#region properties

		[AutoGuiAttr("angular v", false, "G8")]
		public double AngularVelocity { get { return angular_velocity; } }

		[AutoGuiAttr("output", false, "G8")]
		public double Output { get { return output; } }

		[AutoSerializableAttr("KP")]
		[AutoGuiAttr("KP", true, "G8")]
		public double KP { get { return pid.KP; } set { pid.KP = value; } }

		[AutoSerializableAttr("KI")]
		[AutoGuiAttr("KI", true, "G8")]
		public double KI { get { return pid.KI; } set { pid.KI = value; } }

		[AutoSerializableAttr("KD")]
		[AutoGuiAttr("KD", true, "G8")]
		public double KD { get { return pid.KD; } set { pid.KD = value; } }

		[AutoSerializableAttr("AccumulatorClamp")]
		[AutoGuiAttr("AccumulatorClamp", true, "G8")]
		public double AccumulatorClamp { get { return pid.AccumulatorClamp; } set { pid.AccumulatorClamp = value; } }

		[AutoSerializableAttr("AccumulDerivClamp")]
		[AutoGuiAttr("AccumulDerivClamp", true, "G8")]
		public double AccumulDerivClamp { get { return pid.AccumulDerivClamp; } set { pid.AccumulDerivClamp = value; } }

		[AutoSerializableAttr("IntegralClamp")]
		[AutoGuiAttr("IntegralClamp", true, "G8")]
		public double IntegralClamp { get { return pid.IntegralClamp; } set { pid.IntegralClamp = value; } }

		#endregion
	}
}
