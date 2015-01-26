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

    public static class ControlUtils
    {
        public const int PITCH = 0;
        public const int ROLL = 1;
        public const int YAW = 2;

        public static double get_neutralized_user_input(FlightCtrlState state, int axis)
        {
            switch (axis)
            {
                case PITCH:
                    return state.pitch - state.pitchTrim;
                case ROLL:
                    return state.roll - state.rollTrim;
                case YAW:
                    return state.yaw - state.yawTrim;
                default:
                    return 0.0;
            }
        }

        public static void neutralize_user_input(FlightCtrlState state, int axis)
        {
            switch (axis)
            {
                case PITCH:
                    state.pitch = state.pitchTrim;
                    break;
                case ROLL:
                    state.roll = state.rollTrim;
                    break;
                case YAW:
                    state.yaw = state.yawTrim;
                    break;
            }
        }

        public static void set_raw_output(FlightCtrlState state, int axis, double output)
        {
            switch (axis)
            {
                case PITCH:
                    state.pitch = (float)output;
                    state.pitchTrim = 0.0f;
                    break;
                case ROLL:
                    state.roll = (float)output;
                    state.rollTrim = 0.0f;
                    break;
                case YAW:
                    state.yaw = (float)output;
                    state.yawTrim = 0.0f;
                    break;
            }
        }

        public static void set_trim(int axis, InstantControlModel model)
        {
            switch (axis)
            {
                case PITCH:
                    FlightInputHandler.state.pitchTrim = (float)model.input_buf[axis].Average();
                    break;
                case ROLL:
                    FlightInputHandler.state.rollTrim = (float)model.input_buf[axis].Average();
                    break;
                case YAW:
                    FlightInputHandler.state.yawTrim = (float)model.input_buf[axis].Average();
                    break;
            }
        }
    }
}
