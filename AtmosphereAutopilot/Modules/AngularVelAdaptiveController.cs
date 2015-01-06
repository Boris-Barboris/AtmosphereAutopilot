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
	class AngularVelController : AdaptivePIDController
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
		public AngularVelController(Vessel vessel, string module_name,
			int wnd_id, int axis, InstantControlModel model, MediumFlightModel mmodel, AngularAccAdaptiveController acc)
			: base(vessel, module_name, wnd_id)
		{
			this.axis = axis;
			this.model = model;
            this.mmodel = mmodel;
			default_pid_values();
            acc_controller = acc;
		}

		void default_pid_values()
		{
			switch (axis)
			{
				case PITCH:
					pid.KP = 3.0;
					pid.KI = 10.0;
					pid.KD = 0.1;
					pid.IntegralClamp = 0.3;
					pid.AccumulatorClamp = 0.01;
					pid.AccumulDerivClamp = 0.033;
					break;
				case ROLL:
					pid.KP = 3.0;
					pid.KI = 10.0;
					pid.KD = 0.1;
					pid.IntegralClamp = 0.3;
					pid.AccumulatorClamp = 0.01;
					pid.AccumulDerivClamp = 0.033;
					break;
				case YAW:
					pid.KP = 5.0;
					pid.KI = 0.5;
					pid.KD = 0.1;
					pid.IntegralClamp = 0.3;
					pid.AccumulatorClamp = 0.01;
					pid.AccumulDerivClamp = 0.033;
					break;
			}
		}


		[GlobalSerializable("max_part_acceleration")]
		[AutoGuiAttr("max part accel", true, "G8")]
		public double max_part_acceleration = 30.0;			// approx 3g

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
            acc_controller.Activate();
			cycle_counter = 0;
            pid.clear();
		}

		protected override void OnDeactivate()
		{
			vessel.OnAutopilotUpdate -= new FlightInputCallback(ApplyControl);
            acc_controller.Deactivate();
		}

		int cycle_counter = -1;
		bool in_regime = false;
		double time_in_regime = 0.0;
        //bool accumulator_preset = false;

        public override double ApplyControl(FlightCtrlState cntrl, double target_value)
        {
            throw new NotImplementedException();
        }

		/// <summary>
		/// Main control function
		/// </summary>
		/// <param name="cntrl">Control state to change</param>
		public override void ApplyControl(FlightCtrlState cntrl)
		{
			cycle_counter++;
			if (cycle_counter % 500 == 1)		// every 500 OnFixUpdate's recalculate limits
				calculate_limits();

			if (cntrl.killRot)					// skip if ASAS is enabled
				return;

            if (vessel.checkLanded() && axis != YAW)           // ground breaks the model
            {
                in_regime = false;
                time_in_regime = 0.0;
                return;
            }

			input = model.angular_v[axis].getLast();				// get angular velocity
			double accel = model.angular_dv[axis].getLast();		// get angular acceleration
            current_acc = accel;

            // Adapt KP, so that on small value it produces max_input * kp_acc factor output
            pid.KP = kp_acc_factor * max_input_deriv / small_value;
            // Adapt KI
            pid.IntegralClamp = small_value;
            pid.AccumulatorClamp = pid.IntegralClamp * integral_fill_time;
            pid.AccumulDerivClamp = pid.AccumulatorClamp / 3.0 / integral_fill_time;
            pid.KI = ki_koeff * max_input_deriv / pid.AccumulatorClamp;
            // Adapt KD
            pid.KD = kd_kp_koeff * pid.KP;

            double raw_output = pid.Control(input, accel, 0.0, TimeWarp.fixedDeltaTime);

            double error = 0.0 - input;
            proport = error * pid.KP;
            integr = pid.Accumulator * pid.KI;
            deriv = -pid.KD * accel;

            double child_output = acc_controller.ApplyControl(cntrl, raw_output);

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

            output = child_output;
			set_output(cntrl, output);

			if (time_in_regime >= 1.0)
				set_trim();
		}

        [AutoGuiAttr("DEBUG proport", false, "G8")]
        public double proport { get; private set; }

        [AutoGuiAttr("DEBUG integr", false, "G8")]
        public double integr { get; private set; }

        [AutoGuiAttr("DEBUG deriv", false, "G8")]
        public double deriv { get; private set; }

        [AutoGuiAttr("DEBUG prev_control", false, "G8")]
        public double prev_control { get { return model.input_buf[axis].getLast(); } }

        [AutoGuiAttr("DEBUG current_raw", false, "G8")]
        public double current_raw { get; private set; }

        [AutoGuiAttr("DEBUG child_raw", false, "G8")]
        public double child_raw { get; private set; }

        [AutoGuiAttr("DEBUG desired_acc", false, "G8")]
        public double desired_acc { get; private set; }

        [AutoGuiAttr("DEBUG current_acc", false, "G8")]
        public double current_acc { get; private set; }

        [AutoGuiAttr("DEBUG d_control", false, "G8")]
        public double d_control { get; private set; }

        [AutoGuiAttr("DEBUG d_accumulator", false, "G8")]
        public double d_accumulator { get; private set; }

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

        protected override void OnGUICustom()
        {
            acc_controller.ShowGUI();
            acc_controller.OnGUI();
        }

        protected override void BeforeSerialize()
        {
            acc_controller.Serialize();
        }

        protected override void BeforeDeserialize()
        {
            acc_controller.Deserialize();
        }

        [GlobalSerializable("ki_koeff")]
        [AutoGuiAttr("ki_koeff", true, "G6")]
        public double ki_koeff = 0.8;	        // maximum output derivative, simulates control surface reaction speed

		[GlobalSerializable("small_value")]
		[AutoGuiAttr("small value", true, "G6")]
		public double small_value = 0.1;		// arbitrary small input value. Purely intuitive

		[GlobalSerializable("pid_coeff_inertia")]
		[AutoGuiAttr("PID inertia", true, "G6")]
		public double pid_coeff_inertia = 30.0;		// PID coeffitients inertia factor

		[GlobalSerializable("kp_acc_factor")]
		[AutoGuiAttr("KP acceleration factor", true, "G6")]
		public double kp_acc_factor = 0.5;

		[GlobalSerializable("integral_fill_time")]
		[AutoGuiAttr("Integral fill time", true, "G6")]
		public double integral_fill_time = 1.0;

        [GlobalSerializable("kd_kp_koeff")]
        [AutoGuiAttr("KD/KP ratio", true, "G6")]
        public double kd_kp_koeff = 0.33;
	}





    class PitchAngularVelocityController : AngularVelController
    {
        public PitchAngularVelocityController(Vessel vessel, InstantControlModel model, MediumFlightModel mmodel, AngularAccAdaptiveController acc)
            : base(vessel, "Adaptive elavator trimmer", 1234444, 0, model, mmodel, acc)
        { }
    }

}
