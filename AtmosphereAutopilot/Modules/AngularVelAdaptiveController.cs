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
	class AngularVelAdaptiveController : AdaptivePIDController
	{
		public const int PITCH = 0;
		public const int ROLL = 1;
		public const int YAW = 2;

		protected int axis;

		InstantControlModel model;
        MediumFlightModel mmodel;
        AngularAccAdaptiveController acc_controller;

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

		protected override void OnActivate()
		{
            pid.clear();
		}

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
            current_acc = accel;

            // Adapt KP, so that on max_angular_v it produces max_angular_dv * kp_acc factor output
            if (mmodel.max_angular_v[axis] != 0.0)
                pid.KP = kp_acc_factor * mmodel.max_angular_dv[axis] / mmodel.max_angular_v[axis];
            // Adapt KI
            if (integral_fill_time > 1e-3)
            {
                pid.IntegralClamp = mmodel.max_angular_v[axis];
                pid.AccumulatorClamp = pid.IntegralClamp * integral_fill_time;
                pid.AccumulDerivClamp = pid.AccumulatorClamp / 3.0 / integral_fill_time;
                pid.KI = ki_koeff * mmodel.max_angular_dv[axis] / pid.AccumulatorClamp;
            }
            // Adapt KD
            pid.KD = kd_kp_koeff * pid.KP;

            if (FlyByWire)
            {
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
                        max_g = fbw_g_k * 100.0 / (mmodel.max_lever_arm + 1.0);
                        double g_relation = 1.0;
                        double aoa_relation = 1.0;
                        if (max_g > 1e-3 && cur_g >= 0.0)
                            g_relation = cur_g / max_g;
                        if (fbw_max_aoa > 2.0)
                        {
                            const double dgr_to_rad = 1.0 / 180.0 * Math.PI;
                            double max_aoa_rad = fbw_max_aoa * dgr_to_rad;
                            aoa_relation = Math.Abs(mmodel.aoa_pitch.getLast()) / max_aoa_rad +
                                fbw_daoa_k * Common.derivative1_short(
                                    Math.Abs(mmodel.aoa_pitch.getFromTail(1)),
                                    Math.Abs(mmodel.aoa_pitch.getLast()), TimeWarp.fixedDeltaTime) / max_aoa_rad;
                        }
                        fbw_modifier = Common.Clamp(1.0 - Math.Max(aoa_relation, g_relation), 1.0);
                        desired_v *= fbw_modifier;
                    }
                }
                output = Common.Clamp(pid.Control(input, accel, desired_v, TimeWarp.fixedDeltaTime), fbw_dv_k * mmodel.max_angular_dv[axis]);
            }
            else
            {
                desired_v = 0.0;
                output = pid.Control(input, accel, desired_v, TimeWarp.fixedDeltaTime);
            }

            acc_controller.FlyByWire = FlyByWire;

            error = desired_v - input;
            proport = error * pid.KP;
            integr = pid.Accumulator * pid.KI;
            deriv = -pid.KD * accel;

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

        [GlobalSerializable("FlyByWire")]
        [AutoGuiAttr("Fly-By-Wire", true, "G6")]
        public bool FlyByWire = false;

        [GlobalSerializable("fbw_v_k")]
        [VesselSerializable("fbw_v_k")]
        [AutoGuiAttr("moderation v k", true, "G6")]
        public double fbw_v_k = 1.0;

        [GlobalSerializable("fbw_dv_k")]
        [VesselSerializable("fbw_dv_k")]
        [AutoGuiAttr("moderation dv k", true, "G6")]
        public double fbw_dv_k = 1.0;

        [GlobalSerializable("fbw_g_k")]
        [VesselSerializable("fbw_g_k")]
        [AutoGuiAttr("max g-force k", true, "G6")]
        public double fbw_g_k = 1.0;

        [GlobalSerializable("fbw_daoa_k")]
        [VesselSerializable("fbw_daoa_k")]
        [AutoGuiAttr("moderation dAoA k", true, "G6")]
        public double fbw_daoa_k = 0.1;

        [GlobalSerializable("fbw_max_aoa")]
        [VesselSerializable("fbw_max_aoa")]
        [AutoGuiAttr("max AoA degrees", true, "G6")]
        public double fbw_max_aoa = 15.0;

        [AutoGuiAttr("DEBUG proport", false, "G8")]
        public double proport { get; private set; }

        [AutoGuiAttr("DEBUG relative_input", false, "G8")]
        public double desired_v;

        [AutoGuiAttr("DEBUG g_fwb_modifier", false, "G8")]
        public double fbw_modifier;

        [AutoGuiAttr("DEBUG max_g", false, "G8")]
        public double max_g;

        [AutoGuiAttr("DEBUG integr", false, "G8")]
        public double integr { get; private set; }

        [AutoGuiAttr("DEBUG deriv", false, "G8")]
        public double deriv { get; private set; }

        [AutoGuiAttr("DEBUG current_acc", false, "G8")]
        public double current_acc { get; private set; }

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

		void set_output(FlightCtrlState state, double output)
		{
			switch (axis)
			{
				case PITCH:
					state.pitch = (float)output;
					break;
				case ROLL:
					state.roll = (float)output;
					break;
				case YAW:
					state.yaw = (float)output;
					break;
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

        [GlobalSerializable("ki_koeff")]
        [AutoGuiAttr("ki_koeff", true, "G6")]
        public double ki_koeff = 0.0;	        // maximum integral authority

		[GlobalSerializable("kp_acc_factor")]
		[AutoGuiAttr("KP acceleration factor", true, "G6")]
		public double kp_acc_factor = 0.5;

		[GlobalSerializable("integral_fill_time")]
		[AutoGuiAttr("Integral fill time", true, "G6")]
		public double integral_fill_time = 1.0;

        [GlobalSerializable("kd_kp_koeff")]
        [AutoGuiAttr("KD/KP ratio", true, "G6")]
        public double kd_kp_koeff = 0.0;
	}






    class PitchAngularVelocityController : AngularVelAdaptiveController
    {
        public PitchAngularVelocityController(Vessel vessel, InstantControlModel model, MediumFlightModel mmodel, AngularAccAdaptiveController acc)
            : base(vessel, "Adaptive elavator trimmer", 1234444, 0, model, mmodel, acc)
        { }
    }

	class RollAngularVelocityController : AngularVelAdaptiveController
	{
		public RollAngularVelocityController(Vessel vessel, InstantControlModel model, MediumFlightModel mmodel, AngularAccAdaptiveController acc)
			: base(vessel, "Adaptive roll trimmer", 1234445, 1, model, mmodel, acc)
		{ }
	}

	class YawAngularVelocityController : AngularVelAdaptiveController
	{
		public YawAngularVelocityController(Vessel vessel, InstantControlModel model, MediumFlightModel mmodel, AngularAccAdaptiveController acc)
			: base(vessel, "Adaptive yaw trimmer", 1234446, 2, model, mmodel, acc)
		{ }
	}

}
