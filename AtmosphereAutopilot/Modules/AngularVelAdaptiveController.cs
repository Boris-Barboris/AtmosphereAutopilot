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

		/// <summary>
		/// Create controller instance
		/// </summary>
		/// <param name="vessel">Vessel to control</param>
		/// <param name="module_name">Name of controller</param>
		/// <param name="wnd_id">unique for types window id</param>
		/// <param name="axis">Pitch = 0, roll = 1, yaw = 2</param>
		/// <param name="model">Flight model instance for adaptive control</param>
		public AngularVelAdaptiveController(Vessel vessel, string module_name,
			int wnd_id, int axis, InstantControlModel model)
			: base(vessel, module_name, wnd_id)
		{
			this.axis = axis;
			this.model = model;
			default_pid_values();
		}

		void default_pid_values()
		{
			switch (axis)
			{
				case PITCH:
					pid.KP = -3.0;
					pid.KI = -10.0;
					pid.KD = -0.1;
					pid.IntegralClamp = 0.3;
					pid.AccumulatorClamp = 0.01;
					pid.AccumulDerivClamp = 0.033;
					break;
				case ROLL:
					pid.KP = -3.0;
					pid.KI = -10.0;
					pid.KD = -0.1;
					pid.IntegralClamp = 0.3;
					pid.AccumulatorClamp = 0.01;
					pid.AccumulDerivClamp = 0.033;
					break;
				case YAW:
					pid.KP = -5.0;
					pid.KI = -0.5;
					pid.KD = -0.1;
					pid.IntegralClamp = 0.3;
					pid.AccumulatorClamp = 0.01;
					pid.AccumulDerivClamp = 0.033;
					break;
			}
		}


		[GlobalSerializable("max_part_acceleration")]
		[AutoGuiAttr("max part accel", true, "G8")]
		public double max_part_acceleration = 10.0;			// approx 1g

		[AutoGuiAttr("lever arm", false, "G8")]
		public double lever_arm = 1.0;

		void calculate_limits()
		{
			lever_arm = max_part_offset_from_com();
			max_input = Math.Sqrt(Math.Abs(max_part_acceleration) / lever_arm);
			min_input = -max_input;
			max_input_deriv = max_part_acceleration / lever_arm;
			min_input_deriv = -max_input_deriv;
		}

		double max_part_offset_from_com()
		{
			double max_offset = 1.0;
			Vector3 com = vessel.findWorldCenterOfMass();
			foreach (var part in vessel.Parts)
			{
				Vector3 part_v = part.transform.position - com;
				double offset = 0.0;
				switch (axis)
				{
					case PITCH:
                        offset = Vector3.Cross(part_v, vessel.transform.right).magnitude;
						break;
					case ROLL:
                        offset = Vector3.Cross(part_v, vessel.transform.up).magnitude;
						break;
					case YAW:
                        offset = Vector3.Cross(part_v, vessel.transform.forward).magnitude;
						break;
				}
				if (offset > max_offset)
					max_offset = offset;
			}
			return max_offset;
		}

		protected override void OnActivate()
		{
			vessel.OnAutopilotUpdate += new FlightInputCallback(ApplyControl);
			cycle_counter = 0;
            pid.clear();
		}

		protected override void OnDeactivate()
		{
			vessel.OnAutopilotUpdate -= new FlightInputCallback(ApplyControl);
		}

		int cycle_counter = -1;
		bool in_regime = false;
		double time_in_regime = 0.0;

		/// <summary>
		/// Main control function
		/// </summary>
		/// <param name="cntrl">Control state to change</param>
		public override void ApplyControl(FlightCtrlState cntrl)
		{
			cycle_counter++;
			if (cycle_counter % 500 == 1)		// every 5000 OnFixUpdate's recalculate limits
				calculate_limits();

			if (cntrl.killRot)					// skip if ASAS is enabled
				return;

			input = -model.angular_v[axis].getLast();				// get angular velocity
			double accel = -model.angular_dv[axis].getLast();		// get angular acceleration
			double control_authority = -model.k_control[axis];
			double raw_output = 0.0;								// raw unclamped and unsmoothed output

            raw_output = pid.Control(input, accel, 0.0, TimeWarp.fixedDeltaTime);

            double error = 0.0 - input;
            proport = error * pid.KP;
            integr = pid.Accumulator * pid.KI;
            deriv = -pid.KD * pid.InputDerivative;
            derivmodel = -pid.KD * accel;

			if (is_user_handling(cntrl))
			{
				// user is flying
				raw_output = get_user_input(cntrl);
				// apply dampener output
				double dampening = -pid.KD * accel;
				raw_output = raw_output + dampening;
				output = smooth_and_clamp(raw_output);
				set_output(cntrl, output);
				pid.clear();
				return;
			}

			// check if we're stable on given input value
			if (Math.Abs(input) < 1e-2)
			{
				in_regime = true;
				time_in_regime += TimeWarp.fixedDeltaTime;
			}
			else
			{
				in_regime = false;
				time_in_regime = 0.0;
			}

			//
			// Auto-tune PID controller
			//
            if (control_authority > 0.05)			// if control authority is correct
            {
                if (Math.Abs(input) > pid.IntegralClamp)
                {
                    // if current angular velocity is large
                    // we mostly operate with KP here
                    double d2_sign = input *
                        (model.angular_dv[axis].getLast() - model.angular_dv[axis].getFromTail(1));
                    if (d2_sign > 0.0)
                    {
                        // KP is too small, plane is unstable and error is raising
                        pid.KP = apply_with_inertia(pid.KP, pid.KP * pid_coeff_increment * 1.5, pid_coeff_inertia);
                    }
                    else
                    {
                        // Plane is stable and error is decreasing. Need to tune KP
                        if (accel < min_input_deriv || accel > max_input_deriv)
                        {
                            // angular acceleration is too large, need to decrease KP
                            pid.KP = apply_with_inertia(pid.KP, pid.KP / pid_coeff_increment, pid_coeff_inertia);
                        }
                    }
                }

                if (accel > max_input_deriv)
                {
                    double d2_sign = input *
                        (model.angular_dv[axis].getLast() - model.angular_dv[axis].getFromTail(1));
                    if (d2_sign > 0.0)
                    {
                        // Our KD is not enough, system is too unstable
                        pid.KD = apply_with_inertia(pid.KD, pid.KD * pid_coeff_increment * 1.5, pid_coeff_inertia);
                    }
                    else
                    {
                        // Our PD is too large
                        pid.KD = apply_with_inertia(pid.KD, pid.KD / pid_coeff_increment, pid_coeff_inertia);
                    }
                }
            }

			output = smooth_and_clamp(raw_output);
			set_output(cntrl, output);

			if (time_in_regime >= 1.0)
				set_trim(output);
		}

        [AutoGuiAttr("DEBUG proport", false, "G8")]
        public double proport { get; private set; }

        [AutoGuiAttr("DEBUG integr", false, "G8")]
        public double integr { get; private set; }

        [AutoGuiAttr("DEBUG deriv", false, "G8")]
        public double deriv { get; private set; }

        [AutoGuiAttr("DEBUG deriv model", false, "G8")]
        public double derivmodel { get; private set; }

        [AutoGuiAttr("DEBUG prev_control", false, "G8")]
        public double prev_control { get { return model.input_buf[axis].getLast(); } }

        [AutoGuiAttr("DEBUG current_raw", false, "G8")]
        public double current_raw { get; private set; }

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

		void set_trim(double output)
		{
			switch (axis)
			{
				case PITCH:
					FlightInputHandler.state.pitchTrim = (float)output;
					break;
				case ROLL:
					FlightInputHandler.state.rollTrim = (float)output;
					break;
				case YAW:
					FlightInputHandler.state.yawTrim = (float)output;
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

		static double apply_with_inertia(double old, double new_one, double inertia)
		{
			return (new_one + inertia * old) / (inertia + 1.0);
		}

		[GlobalSerializable("max_output_deriv")]
		[AutoGuiAttr("csurface speed", true, "G6")]
		public double max_output_deriv = 20.0;	// maximum output derivative, simulates control surface reaction speed

		[GlobalSerializable("prop_relax_desire")]
		[AutoGuiAttr("proport relax t", true, "G6")]
		public double prop_relax_desire = 2.0;		// desired time of proportional relaxation

		public double small_value = 0.05;			// arbitrary small input value. Purely intuitive

		[GlobalSerializable("pid_coeff_inertia")]
		[AutoGuiAttr("PID inertia", true, "G6")]
		public double pid_coeff_inertia = 20.0;		// PID coeffitients inertia factor

		[GlobalSerializable("pid_coeff_increment")]
		[AutoGuiAttr("PID increment", true, "G6")]
		public double pid_coeff_increment = 1.05;	// PID coeffitients increment factor
	}
}
