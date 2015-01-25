using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;
using System.IO;

namespace AtmosphereAutopilot
{
	/// <summary>
    /// Controls angular acceleration. Meant to be created from AngularVelAdaptiveController
	/// </summary>
	public class AngularAccAdaptiveController : StateController
	{
		protected int axis;

		InstantControlModel model;
        MediumFlightModel m_model;

        StreamWriter errorWriter, controlWriter, v_writer, dv_writer, smooth_dv_writer, desire_dv_writer, extr_dv_writer;

		
		PIController pid = new PIController();

		#region PIproperties

		[AutoGuiAttr("Accumulator", false, "G8")]
		internal double Accumulator { get { return pid.Accumulator; } }

		[AutoGuiAttr("KP", false, "G8")]
		internal double KP { get { return pid.KP; } }

		[AutoGuiAttr("KI", false, "G8")]
		internal double KI { get { return pid.KI; } }

		[AutoGuiAttr("AccumulatorClamp", false, "G8")]
		internal double AccumulatorClamp { get { return pid.AccumulatorClamp; } }

		[AutoGuiAttr("AccumulDerivClamp", false, "G8")]
		internal double AccumulDerivClamp { get { return pid.AccumulDerivClamp; } }

		[AutoGuiAttr("IntegralClamp", false, "G8")]
		internal double IntegralClamp { get { return pid.IntegralClamp; } }

		[AutoGuiAttr("IntegralGain", false, "G8")]
		internal double IntegralGain { get { return pid.IntegralGain; } }

		#endregion


		/// <summary>
		/// Create controller instance.
		/// </summary>
		/// <param name="vessel">Vessel to control</param>
		/// <param name="module_name">Name of controller</param>
		/// <param name="wnd_id">unique for types window id</param>
		/// <param name="axis">Pitch = 0, roll = 1, yaw = 2</param>
		/// <param name="model">Flight model instance for adaptive control</param>
        /// <param name="parent">AngularVelAdaptiveController wich uses this instance as a child</param>
        internal AngularAccAdaptiveController(Vessel vessel, string module_name,
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

        protected override void OnDeactivate()
        {
			write_telemetry = false;
        }

        public override void ApplyControl(FlightCtrlState cntrl)
        {
            throw new NotImplementedException();
        }

        int write_cycle = 0;

		/// <summary>
		/// Main control function
		/// </summary>
		/// <param name="cntrl">Control state to change</param>
        /// <param name="target_value">Desired angular acceleration</param>
		public override double ApplyControl(FlightCtrlState cntrl, double target_value)
		{
            input = model.angular_dv[axis].getLast();
            desired_acc = target_value;

            error = target_value - input;

            if (write_telemetry)
            {
                errorWriter.Write(error.ToString("G8") + ',');
                if (write_cycle >= 3)
                    smooth_dv_writer.Write(model.angular_dv_central[axis].getLast().ToString("G8") + ',');
                else
                    write_cycle++;
                desire_dv_writer.Write(target_value.ToString("G8") + ',');
                dv_writer.Write(input.ToString("G8") + ',');
                extr_dv_writer.Write(input.ToString("G8") + ',');
                v_writer.Write(model.angular_v[axis].getLast().ToString("G8") + ',');
                controlWriter.Write(model.input_buf[axis].getLast().ToString("G8") + ',');
            }
            else
                write_cycle = 0;

            double auth = k_auth;
            double mistake_avg = model.dv_mistake[axis].Average();
            small_value_low = small_value_k_low * mistake_avg;
            small_value_high = small_value_k_high * mistake_avg;
            if (auth > 1e-5)
            {
                if (Math.Abs(error) > small_value_low)
                {
                    double cutoff_smoothing = Common.Clamp((Math.Abs(error) - small_value_low) / 
                        (small_value_high - small_value_low), 0.0, 1.0);
                    pid.KP = cutoff_smoothing * kp_koeff / auth;
                }
                else
                {
                    pid.KP = 0.0;
                }
            }

            large_value = m_model.max_angular_dv[axis];
            if (integral_fill_time > 1e-3 && large_value > 1e-3)
            {
                pid.IntegralClamp = large_value * large_value_k;
                pid.AccumulatorClamp = pid.IntegralClamp * integral_fill_time;
                pid.AccumulDerivClamp = pid.AccumulatorClamp / integral_fill_time;
                pid.KI = ki_koeff / pid.AccumulatorClamp;
                // clamp gain on small errors
                pid.IntegralGain = Common.Clamp(i_gain + Math.Abs(error) * (1.0 - i_gain) / large_value, 1.0);
            }

            current_raw = pid.Control(input, target_value, TimeWarp.fixedDeltaTime);

            proport = error * pid.KP;
            integr = pid.Accumulator * pid.KI;

			output = Common.Clamp(current_raw, 1.0);
            set_output(cntrl, output);
            return output;
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

        [AutoGuiAttr("Write telemetry", true, "G8")]
        protected bool write_telemetry 
		{
			get { return _write_telemetry; }
			set
			{
				if (value)
				{
                    if (!_write_telemetry)
                    {
                        errorWriter = File.CreateText("D:/Games/Kerbal Space Program 0.90/Resources/" +
                            vessel.name + '_' + module_name + "_telemetry_error.csv");
                        controlWriter = File.CreateText("D:/Games/Kerbal Space Program 0.90/Resources/" +
                            vessel.name + '_' + module_name + "_telemetry_control.csv");
                        v_writer = File.CreateText("D:/Games/Kerbal Space Program 0.90/Resources/" +
                            vessel.name + '_' + module_name + "_telemetry_v.csv");
                        dv_writer = File.CreateText("D:/Games/Kerbal Space Program 0.90/Resources/" +
                            vessel.name + '_' + module_name + "_telemetry_dv.csv");
                        desire_dv_writer = File.CreateText("D:/Games/Kerbal Space Program 0.90/Resources/" +
                            vessel.name + '_' + module_name + "_telemetry_desire.csv");
                        extr_dv_writer = File.CreateText("D:/Games/Kerbal Space Program 0.90/Resources/" +
                            vessel.name + '_' + module_name + "_telemetry_extrdv.csv");
                        smooth_dv_writer = File.CreateText("D:/Games/Kerbal Space Program 0.90/Resources/" +
                            vessel.name + '_' + module_name + "_telemetry_smoothdv.csv");
                        desire_dv_writer.Write("0.0,");
                        errorWriter.Write(error.ToString("G8") + ',');
                        extr_dv_writer.Write("0.0,");
                        _write_telemetry = value;
                    }
				}
				else
				{
                    if (_write_telemetry)
                    {
                        errorWriter.Close();
                        controlWriter.Close();
                        v_writer.Close();
                        dv_writer.Close();
                        desire_dv_writer.Close();
                        extr_dv_writer.Close();
                        smooth_dv_writer.Close();
                        _write_telemetry = value;
                    }					
				}
			}
		}
		bool _write_telemetry = false;


		#region Parameters

		[AutoGuiAttr("DEBUG proport", false, "G8")]
        internal double proport { get; private set; }

        [AutoGuiAttr("DEBUG integr", false, "G8")]
        internal double integr { get; private set; }

        [AutoGuiAttr("DEBUG current_raw", false, "G8")]
        internal double current_raw { get; private set; }

        [AutoGuiAttr("DEBUG desired_dv", false, "G8")]
        internal double desired_acc { get; private set; }

        [AutoGuiAttr("DEBUG current_d2v", false, "G8")]
        internal double current_d2v { get; private set; }

        [AutoGuiAttr("DEBUG authority", false, "G8")]
        internal double k_auth { get { return model.getDvAuthority((Axis)axis); } }

        [GlobalSerializable("user_dampening")]
        [AutoGuiAttr("user_dampening", true, "G6")]
        protected double user_dampening = 1.0;

		[GlobalSerializable("pid_coeff_inertia")]
		[AutoGuiAttr("PID inertia", true, "G6")]
		protected double pid_coeff_inertia = 15.0;		// PID coeffitients inertia factor

		[GlobalSerializable("ki_koeff")]
		[AutoGuiAttr("ki_koeff", true, "G6")]
		protected double ki_koeff = 0.8;

        [GlobalSerializable("kp_koeff")]
        [AutoGuiAttr("KP/Authority ratio", true, "G6")]
        protected double kp_koeff = 0.75;

		[AutoGuiAttr("large value", false, "G6")]
		protected double large_value = 10.0;

        [GlobalSerializable("large_value_k")]
        [AutoGuiAttr("large value k", true, "G6")]
        protected double large_value_k = 5.0;

        [AutoGuiAttr("small value high", false, "G6")]
		protected double small_value_high = 0.1;

		[AutoGuiAttr("small value low", false, "G6")]
		protected double small_value_low = 0.1;

        [GlobalSerializable("small_value_k_high")]
        [AutoGuiAttr("small value k high", true, "G6")]
        protected double small_value_k_high = 1.5;

        [GlobalSerializable("small_value_k_low")]
        [AutoGuiAttr("small value k low", true, "G6")]
        protected double small_value_k_low = 0.5;

        [GlobalSerializable("integral_fill_time")]
        [AutoGuiAttr("Integral fill time", true, "G6")]
        protected double integral_fill_time = 0.1;

        [GlobalSerializable("i_gain")]
        [AutoGuiAttr("Equilibrium i_gain", true, "G6")]
        protected double i_gain = 1.0;

        [GlobalSerializable("i_overshoot_gain")]
        [AutoGuiAttr("Integral overshoot gain", true, "G6")]
        protected double i_overshoot_gain = 1.0;

		#endregion
	}


	//
	// Three realizations
	//

    public sealed class PitchAngularAccController : AngularAccAdaptiveController
    {
        internal PitchAngularAccController(Vessel vessel, InstantControlModel model, MediumFlightModel mmodel)
            : base(vessel, "Adaptive elavator trimmer accel", 77821329, (int)Axis.PITCH, model, mmodel)
        { }
    }

	public sealed class RollAngularAccController : AngularAccAdaptiveController
	{
		internal RollAngularAccController(Vessel vessel, InstantControlModel model, MediumFlightModel mmodel)
			: base(vessel, "Adaptive roll trimmer accel", 77821330, (int)Axis.ROLL, model, mmodel)
		{ }
	}

	public sealed class YawAngularAccController : AngularAccAdaptiveController
	{
		internal YawAngularAccController(Vessel vessel, InstantControlModel model, MediumFlightModel mmodel)
			: base(vessel, "Adaptive yaw trimmer accel", 77821331, (int)Axis.YAW, model, mmodel)
		{ }
	}

}
