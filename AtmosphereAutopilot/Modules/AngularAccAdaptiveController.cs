using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;

namespace AtmosphereAutopilot
{
	/// <summary>
    /// Controls angular acceleration. Meant to be created from AngularVelAdaptiveController
	/// </summary>
	class AngularAccAdaptiveController : AdaptivePIDController
	{
		public const int PITCH = 0;
		public const int ROLL = 1;
		public const int YAW = 2;

		protected int axis;

		InstantControlModel model;
        MediumFlightModel m_model;

		/// <summary>
		/// Create controller instance.
		/// </summary>
		/// <param name="vessel">Vessel to control</param>
		/// <param name="module_name">Name of controller</param>
		/// <param name="wnd_id">unique for types window id</param>
		/// <param name="axis">Pitch = 0, roll = 1, yaw = 2</param>
		/// <param name="model">Flight model instance for adaptive control</param>
        /// <param name="parent">AngularVelAdaptiveController wich uses this instance as a child</param>
        public AngularAccAdaptiveController(Vessel vessel, string module_name,
            int wnd_id, int axis, InstantControlModel model, MediumFlightModel m_model)
			: base(vessel, module_name, wnd_id)
		{
			this.axis = axis;
			this.model = model;
            this.m_model = m_model;
		}

		protected override void OnActivate()
		{
            pid.clear();
		}

        protected override void OnDeactivate() { }

        public override void ApplyControl(FlightCtrlState cntrl)
        {
            throw new NotImplementedException();
        }

        // Accumulator persistance section, tries to save accumulator between user input sessions
        bool user_is_controlling = false;
        double last_aoa = 0.0;
        double last_speed = 0.0;

		/// <summary>
		/// Main control function
		/// </summary>
		/// <param name="cntrl">Control state to change</param>
        /// <param name="target_value">Desired angular acceleration</param>
		public override double ApplyControl(FlightCtrlState cntrl, double target_value)
		{
            input = model.angular_dv[axis].getLast();
            current_d2v = InstantControlModel.derivative1_short(
                model.angular_dv[axis].getFromTail(1),
                model.angular_dv[axis].getFromTail(0),
                TimeWarp.fixedDeltaTime);
            desired_acc = target_value;

            double auth = k_auth;
            if (auth > 0.05)
            {
                // authority is meaningfull value
                // adapt KP
                pid.KP = apply_with_inertia(pid.KP, kp_koeff / auth, pid_coeff_inertia);
                // adapt KD
                pid.KD = apply_with_inertia(pid.KD, kp_kd_ratio * pid.KP, pid_coeff_inertia);
            }

            if (integral_fill_time > 1e-3)
            {
                pid.IntegralClamp = small_value;
                pid.AccumulatorClamp = pid.IntegralClamp * integral_fill_time;
                pid.AccumulDerivClamp = pid.AccumulatorClamp / 3.0 / integral_fill_time;
                pid.KI = ki_koeff / pid.AccumulatorClamp;
            }

            if (is_user_handling(cntrl))
            {
                // user is flying
                double raw_output = get_user_input(cntrl);
                // apply dampener output
                double dampening = -user_dampening * pid.KP * input;
                raw_output = raw_output + dampening;
                output = smooth_and_clamp(raw_output);
                set_output(cntrl, output);
                if (!user_is_controlling)
                {
                    if (axis == PITCH)
                        last_aoa = m_model.aoa_pitch.getLast();
                    if (axis == YAW)
                        last_aoa = m_model.aoa_yaw.getLast();
                    last_speed = vessel.srfSpeed;
                    user_is_controlling = true;
                }
                return output;
            }
            else
                if (user_is_controlling)
                {
                    user_is_controlling = false;
                    double accumul_persistance_k = 1.0;
                    if (axis == PITCH)
                        accumul_persistance_k *= Common.Clamp(
                            1 - Math.Abs(last_aoa - m_model.aoa_pitch.getLast()) / 0.05, 0.0, 1.0);
                    if (axis == YAW)
                        accumul_persistance_k *= Common.Clamp(
                            1 - Math.Abs(last_aoa - m_model.aoa_yaw.getLast()) / 0.05, 0.0, 1.0);
                    accumul_persistance_k *= Common.Clamp(1 - Math.Abs(vessel.srfSpeed - last_speed) / 50.0, 0.0, 1.0);
                    pid.Accumulator *= accumul_persistance_k;
                }

            current_raw = pid.Control(input, current_d2v, target_value, TimeWarp.fixedDeltaTime);
            error = target_value - input;
            proport = error * pid.KP;
            integr = pid.Accumulator * pid.KI;
            deriv = -pid.KD * current_d2v;

            output = smooth_and_clamp(current_raw);
            return output;
		}

        bool is_user_handling(FlightCtrlState state)
        {
            switch (axis)
            {
                case PITCH:
                    return !(state.pitch == state.pitchTrim);
                case ROLL:
                    return !(state.roll == state.rollTrim);
                case YAW:
                    return !(state.yaw == state.yawTrim);
                default:
                    return false;
            }
        }

        double get_user_input(FlightCtrlState state)
        {
            switch (axis)
            {
                case PITCH:
                    return state.pitch;
                case ROLL:
                    return state.roll;
                case YAW:
                    return state.yaw;
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

        double smooth_and_clamp(double raw)
        {
            double prev_output = model.input_buf[axis].getLast();	// get previous control input
            double smoothed = raw;
            current_raw = raw;
            double raw_d = (raw - prev_output) / TimeWarp.fixedDeltaTime;
            if (raw_d > max_output_deriv)
                smoothed = prev_output + TimeWarp.fixedDeltaTime * max_output_deriv;
            if (raw_d < -max_output_deriv)
                smoothed = prev_output - TimeWarp.fixedDeltaTime * max_output_deriv;
            return Common.Clamp(smoothed, 1.0);
        }

        [AutoGuiAttr("DEBUG error", false, "G8")]
        public double error { get; private set; }

        [AutoGuiAttr("DEBUG proport", false, "G8")]
        public double proport { get; private set; }

        [AutoGuiAttr("DEBUG integr", false, "G8")]
        public double integr { get; private set; }

        [AutoGuiAttr("DEBUG deriv", false, "G8")]
        public double deriv { get; private set; }

        [AutoGuiAttr("DEBUG current_raw", false, "G8")]
        public double current_raw { get; private set; }

        [AutoGuiAttr("DEBUG desired_dv", false, "G8")]
        public double desired_acc { get; private set; }

        [AutoGuiAttr("DEBUG current_d2v", false, "G8")]
        public double current_d2v { get; private set; }

        [AutoGuiAttr("DEBUG authority", false, "G8")]
        public double k_auth { get { return model.getDvAuthority(axis); } }

        [GlobalSerializable("max_output_deriv")]
        [AutoGuiAttr("csurface speed", true, "G6")]
        public double max_output_deriv = 10.0;	// maximum output derivative, simulates control surface reaction speed

        [GlobalSerializable("user_dampening")]
        [AutoGuiAttr("user_dampening", true, "G6")]
        public double user_dampening = 1.0;

		[GlobalSerializable("pid_coeff_inertia")]
		[AutoGuiAttr("PID inertia", true, "G6")]
		public double pid_coeff_inertia = 15.0;		// PID coeffitients inertia factor

        [GlobalSerializable("ki_koeff")]
        [AutoGuiAttr("ki_koeff", true, "G6")]
        public double ki_koeff = 0.8;	        // maximum output derivative, simulates control surface reaction speed

        [GlobalSerializable("kp_kd_ratio")]
		[AutoGuiAttr("KD/KP ratio", true, "G6")]
		public double kp_kd_ratio = 0.33;

        [GlobalSerializable("kp_koeff")]
        [AutoGuiAttr("KP/Authority ratio", true, "G6")]
        public double kp_koeff = 0.75;

        [GlobalSerializable("small_value")]
        [AutoGuiAttr("small value", true, "G6")]
        public double small_value = 10.0;		    // arbitrary small input value. Purely intuitive

        [GlobalSerializable("integral_fill_time")]
        [AutoGuiAttr("Integral fill time", true, "G6")]
        public double integral_fill_time = 0.1;
	}

    class PitchAngularAccController : AngularAccAdaptiveController
    {
        public PitchAngularAccController(Vessel vessel, InstantControlModel model, MediumFlightModel mmodel)
            : base(vessel, "Adaptive elavator trimmer accel", 77821329, 0, model, mmodel)
        { }
    }

}
