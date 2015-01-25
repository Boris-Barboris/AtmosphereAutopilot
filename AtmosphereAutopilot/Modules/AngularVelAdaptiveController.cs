using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;

namespace AtmosphereAutopilot
{

	/// <summary>
	/// Controls angular velocity
	/// </summary>
	public class AngularVelAdaptiveController : StateController
	{
		protected int axis;

		InstantControlModel model;
        MediumFlightModel mmodel;
        AngularAccAdaptiveController acc_controller;


		PController pid = new PController();

		#region CpntrollerProperties

		[AutoGuiAttr("KP", false, "G8")]
		internal double KP { get { return pid.KP; } }

		#endregion


		/// <summary>
		/// Create controller instance
		/// </summary>
		/// <param name="vessel">Vessel to control</param>
		/// <param name="module_name">Name of controller</param>
		/// <param name="wnd_id">unique for types window id</param>
		/// <param name="axis">Pitch = 0, roll = 1, yaw = 2</param>
		/// <param name="model">Flight model instance for adaptive control</param>
		public AngularVelAdaptiveController(Vessel vessel, string module_name,
			int wnd_id, int axis, InstantControlModel model, MediumFlightModel mmodel, AngularAccAdaptiveController acc)
			: base(vessel, module_name, wnd_id)
		{
			this.axis = axis;
			this.model = model;
            this.mmodel = mmodel;
            acc_controller = acc;
		}

		protected override void OnActivate() { }

        protected override void OnDeactivate() { }

		double time_in_regime = 0.0;

        public override void ApplyControl(FlightCtrlState cntrl)
        {
            throw new NotImplementedException();
        }

		/// <summary>
		/// Main control function
		/// </summary>
		/// <param name="cntrl">Control state to change</param>
        public override double ApplyControl(FlightCtrlState cntrl, double target_value)
		{
			input = model.angular_v[axis].getLast();				// get angular velocity
			double accel = model.angular_dv[axis].getLast();		// get angular acceleration

            // Adapt KP, so that on max_angular_v it produces max_angular_dv * kp_acc factor output
            if (mmodel.max_angular_v[axis] != 0.0)
                pid.KP = kp_acc_factor * mmodel.max_angular_dv[axis] / mmodel.max_angular_v[axis];

            double user_input = get_neutralized_user_input(cntrl);
            if (user_input != 0.0)
                desired_v = fbw_v_k * user_input * mmodel.max_angular_v[axis];      // user is interfering with control
            else
                desired_v = target_value;                                           // control from above
            if (axis == PITCH)
            {
                // limit it due to g-force limitations
                double cur_g = mmodel.g_force.getLast();
                if (desired_v * mmodel.aoa_pitch.getLast() > 0.0)
                {
                    // user is trying to increase AoA
                    max_g = fbw_g_k / mmodel.wing_load_k / mmodel.wing_load_k;
                    double g_relation = 1.0;
                    double aoa_relation = 1.0;
                    if (max_g > 1e-3 && cur_g >= 0.0)
                    {
                        g_relation = cur_g / max_g;
                    }
                    if (fbw_max_aoa > 2.0)
                    {
                        const double dgr_to_rad = 1.0 / 180.0 * Math.PI;
                        double max_aoa_rad = fbw_max_aoa * dgr_to_rad;
                        aoa_relation = Math.Abs(mmodel.aoa_pitch.getLast()) / max_aoa_rad;
                    }
                    double max_k = Math.Max(aoa_relation, g_relation);
                    fbw_modifier = Common.Clamp(1.0 - max_k, 1.0);
                    desired_v *= fbw_modifier;
                }
            }
            output = Common.Clamp(pid.Control(input, desired_v), fbw_dv_k * mmodel.max_angular_dv[axis]);

            error = desired_v - input;
            proport = error * pid.KP;

            acc_controller.ApplyControl(cntrl, output);

			// check if we're stable on given input value
            if (Math.Abs(input) < 5e-3)
			{
				time_in_regime += TimeWarp.fixedDeltaTime;
			}
			else
			{
				time_in_regime = 0.0;
			}

			if (time_in_regime >= 1.0 && axis != YAW)
				set_trim();

            return output;
		}


		#region Parameters

		[GlobalSerializable("fbw_v_k")]
        [VesselSerializable("fbw_v_k")]
        [AutoGuiAttr("moderation v k", true, "G6")]
        protected double fbw_v_k = 1.0;

        [GlobalSerializable("fbw_dv_k")]
        [VesselSerializable("fbw_dv_k")]
        [AutoGuiAttr("moderation dv k", true, "G6")]
        protected double fbw_dv_k = 1.0;

        [GlobalSerializable("fbw_g_k")]
        [VesselSerializable("fbw_g_k")]
        [AutoGuiAttr("max g-force k", true, "G6")]
        protected double fbw_g_k = 1.0;

        [GlobalSerializable("fbw_daoa_k")]
        [VesselSerializable("fbw_daoa_k")]
        [AutoGuiAttr("moderation dAoA k", true, "G6")]
        protected double fbw_daoa_k = 0.1;

        [GlobalSerializable("fbw_max_aoa")]
        [VesselSerializable("fbw_max_aoa")]
        [AutoGuiAttr("max AoA degrees", true, "G6")]
        protected double fbw_max_aoa = 15.0;

        [AutoGuiAttr("DEBUG proport", false, "G8")]
        internal double proport { get; private set; }

        [AutoGuiAttr("DEBUG relative_input", false, "G8")]
        protected double desired_v;

        [AutoGuiAttr("DEBUG g_fwb_modifier", false, "G8")]
        protected double fbw_modifier;

        [AutoGuiAttr("DEBUG max_g", false, "G8")]
        protected double max_g;

		[GlobalSerializable("kp_acc_factor")]
		[AutoGuiAttr("KP acceleration factor", true, "G6")]
		protected double kp_acc_factor = 0.5;

		#endregion


		double get_neutralized_user_input(FlightCtrlState state)
        {
            double result;
            switch (axis)
            {
                case PITCH:
                    result = state.pitch == state.pitchTrim ?
                        0.0 :
                        state.pitch > state.pitchTrim ?
                            (state.pitch - state.pitchTrim) / (1.0 - state.pitchTrim) :
                            (state.pitch - state.pitchTrim) / (1.0 + state.pitchTrim);
                    return result;
                case ROLL:
                    result = state.roll == state.rollTrim ?
                        0.0 :
                        state.roll > state.rollTrim ?
                            (state.roll - state.rollTrim) / (1.0 - state.rollTrim) :
                            (state.roll - state.rollTrim) / (1.0 + state.rollTrim);
                    return result;
                case YAW:
                    result = state.yaw == state.yawTrim ?
                        0.0 :
                        state.yaw > state.yawTrim ?
                            (state.yaw - state.yawTrim) / (1.0 - state.yawTrim) :
                            (state.yaw - state.yawTrim) / (1.0 + state.yawTrim);
                    return result;
                default:
                    return 0.0;
            }
        }

		void set_trim()
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


	//
	// Three realizations
	//

    public sealed class PitchAngularVelocityController : AngularVelAdaptiveController
    {
        internal PitchAngularVelocityController(Vessel vessel, InstantControlModel model, 
			MediumFlightModel mmodel, AngularAccAdaptiveController acc)
            : base(vessel, "Adaptive elavator trimmer", 1234444, 0, model, mmodel, acc)
        { }
    }

	public sealed class RollAngularVelocityController : AngularVelAdaptiveController
	{
		internal RollAngularVelocityController(Vessel vessel, InstantControlModel model, 
			MediumFlightModel mmodel, AngularAccAdaptiveController acc)
			: base(vessel, "Adaptive roll trimmer", 1234445, 1, model, mmodel, acc)
		{ }
	}

	public sealed class YawAngularVelocityController : AngularVelAdaptiveController
	{
		internal YawAngularVelocityController(Vessel vessel, InstantControlModel model, 
			MediumFlightModel mmodel, AngularAccAdaptiveController acc)
			: base(vessel, "Adaptive yaw trimmer", 1234446, 2, model, mmodel, acc)
		{ }
	}

}
