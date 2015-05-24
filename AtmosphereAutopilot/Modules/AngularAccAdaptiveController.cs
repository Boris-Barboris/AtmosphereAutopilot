using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;
using System.IO;

namespace AtmosphereAutopilot
{
	/// <summary>
    /// Controls angular acceleration. Meant to be used by AngularVelAdaptiveController
	/// </summary>
	public abstract class AngularAccAdaptiveController : SIMOController
	{
		protected int axis;

		InstantControlModel imodel;
        MediumFlightModel mmodel;

		// Telemetry writers
		StreamWriter controlWriter, v_writer, acc_writer, prediction_writer, prediction_2_writer, desire_dv_writer, aoa_writer;

		/// <summary>
		/// Create controller instance.
		/// </summary>
		/// <param name="vessel">Vessel to control</param>
		/// <param name="module_name">Name of controller</param>
		/// <param name="wnd_id">unique for types window id</param>
		/// <param name="axis">Pitch = 0, roll = 1, yaw = 2</param>
        protected AngularAccAdaptiveController(Vessel vessel, string module_name,
            int wnd_id, int axis)
			: base(vessel, module_name, wnd_id)
		{
			this.axis = axis;
		}

		public override void InitializeDependencies(Dictionary<Type, AutopilotModule> modules)
		{
			this.imodel = modules[typeof(InstantControlModel)] as InstantControlModel;
			this.mmodel = modules[typeof(MediumFlightModel)] as MediumFlightModel;
		}

		protected override void OnActivate()
		{
            imodel.Activate();
            mmodel.Activate();
		}

        protected override void OnDeactivate()
        {
			write_telemetry = false;
            imodel.Deactivate();
            mmodel.Deactivate();
        }

		/// <summary>
		/// Main control function
		/// </summary>
		/// <param name="cntrl">Control state to change</param>
        /// <param name="target_value">Desired angular acceleration</param>
        public override float ApplyControl(FlightCtrlState cntrl, float target_value)
		{
            input = imodel.AngularAcc(axis);
            desired_acc = target_value;
            error = target_value - input;

            if (write_telemetry)
            {
                desire_dv_writer.Write(target_value.ToString("G8") + ',');
                acc_writer.Write(input.ToString("G8") + ',');
                v_writer.Write(imodel.AngularVel(axis).ToString("G8") + ',');
                prediction_writer.Write(imodel.prediction[axis].ToString("G8") + ',');
                prediction_2_writer.Write(imodel.prediction_2[axis].ToString("G8") + ',');
				aoa_writer.Write(imodel.AoA(axis).ToString("G8") + ',');
            }

            float current_raw = output;
			float predicted_diff = desired_acc - imodel.prediction[axis];
			float required_control_diff = predicted_diff / k_auth / TimeWarp.fixedDeltaTime;

			// dampen control_diff near oscillation regions
			float ETN = Math.Abs(error) / (noise_damp_k * imodel.noiseness[axis]);
			if (ETN < 1.0f)
				required_control_diff *= (float)Math.Pow(ETN, noise_damp_pow);

            output = Common.Clampf(current_raw + Common.Clampf(required_control_diff, max_input_deriv), 1.0f);
            ControlUtils.set_raw_output(cntrl, axis, output);

            if (write_telemetry)
                controlWriter.Write(output.ToString("G8") + ',');

            return output;
		}

        [AutoGuiAttr("Write telemetry", true)]
        protected bool write_telemetry 
		{
			get { return _write_telemetry; }
			set
			{
				if (value)
				{
                    if (!_write_telemetry)
                    {
                        controlWriter = File.CreateText(KSPUtil.ApplicationRootPath + "/Resources/" +
                            vessel.name + '_' + module_name + " control.csv");
                        v_writer = File.CreateText(KSPUtil.ApplicationRootPath + "/Resources/" +
                            vessel.name + '_' + module_name + " v.csv");
                        acc_writer = File.CreateText(KSPUtil.ApplicationRootPath + "/Resources/" +
                            vessel.name + '_' + module_name + " acc.csv");
                        desire_dv_writer = File.CreateText(KSPUtil.ApplicationRootPath + "/Resources/" +
                            vessel.name + '_' + module_name + " desire.csv");
                        prediction_writer = File.CreateText(KSPUtil.ApplicationRootPath + "/Resources/" +
                            vessel.name + '_' + module_name + " predict.csv");
                        prediction_2_writer = File.CreateText(KSPUtil.ApplicationRootPath + "/Resources/" +
                            vessel.name + '_' + module_name + " predict_2.csv");
						aoa_writer = File.CreateText(KSPUtil.ApplicationRootPath + "/Resources/" +
							vessel.name + '_' + module_name + " aoa.csv");
                        prediction_writer.Write("0.0,");
                        prediction_2_writer.Write("0.0,0.0,");
                        _write_telemetry = value;
                    }
				}
				else
				{
                    if (_write_telemetry)
                    {
                        controlWriter.Close();
                        v_writer.Close();
                        acc_writer.Close();
                        desire_dv_writer.Close();
                        prediction_writer.Close();
                        prediction_2_writer.Close();
						aoa_writer.Close();
                        _write_telemetry = value;
                    }					
				}
			}
		}
		bool _write_telemetry = false;


		#region Parameters

        [AutoGuiAttr("DEBUG desired acc", false, "G8")]
        internal float desired_acc { get; private set; }

        [AutoGuiAttr("DEBUG authority", false, "G8")]
        internal float k_auth { get { return imodel.linear_authority[axis]; } }

        [AutoGuiAttr("Control speed limit", true, "G8")]
        [GlobalSerializable("Control speed limit")]
        protected float max_input_deriv = 0.5f;

		[AutoGuiAttr("Noise damp diap", true, "G8")]
		[GlobalSerializable("Noise damp diap")]
		protected float noise_damp_k = 2.0f;

		[AutoGuiAttr("Noise damp power", true, "G8")]
		[GlobalSerializable("Noise damp power")]
		protected float noise_damp_pow = 3.0f;

		#endregion
	}


	//
	// Three realizations
	//

    public sealed class PitchAngularAccController : AngularAccAdaptiveController
    {
        internal PitchAngularAccController(Vessel vessel)
            : base(vessel, "Adaptive elavator trimmer accel", 77821329, PITCH)
        { }
    }

	public sealed class RollAngularAccController : AngularAccAdaptiveController
	{
		internal RollAngularAccController(Vessel vessel)
			: base(vessel, "Adaptive roll trimmer accel", 77821330, ROLL)
		{ }
	}

	public sealed class YawAngularAccController : AngularAccAdaptiveController
	{
		internal YawAngularAccController(Vessel vessel)
			: base(vessel, "Adaptive yaw trimmer accel", 77821331, YAW)
		{ }
	}

}
