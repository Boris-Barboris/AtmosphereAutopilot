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
		StreamWriter controlWriter, v_writer, acc_writer, prediction_writer, 
			desire_acc_writer, aoa_writer, airspd_writer, density_writer;

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
            input = (float)imodel.AngularAcc(axis);
            desired_acc = target_value;
            error = target_value - input;

            if (write_telemetry)
            {
                desire_acc_writer.Write(target_value.ToString("G8") + ',');
                acc_writer.Write(input.ToString("G8") + ',');
                v_writer.Write(imodel.AngularVel(axis).ToString("G8") + ',');
                prediction_writer.Write(imodel.angular_vel[axis].ToString("G8") + ',');
				aoa_writer.Write(imodel.AoA(axis).ToString("G8") + ',');
				airspd_writer.Write((imodel.up_srf_v + imodel.fwd_srf_v).magnitude.ToString("G8") + ',');
				density_writer.Write(vessel.atmDensity.ToString("G8") + ',');
            }

			//float current_raw = output;
			//float predicted_diff = desired_acc - imodel.prediction[axis];
			//float required_control_diff = predicted_diff / (k_auth != 0.0f ? k_auth : 1.0f) / imodel.MOI[axis];

			//output = Common.Clampf(current_raw + Common.Clampf(required_control_diff, max_input_deriv), 1.0f);
            //ControlUtils.set_raw_output(cntrl, axis, output);

			float prev_input = imodel.ControlInput(axis);
			float cur_input_raw = ControlUtils.getControlFromState(cntrl, axis);
			output = cur_input_raw;
			true_output = far_exponential_blend(true_output, output);

			ControlUtils.set_raw_output(cntrl, axis, output);

            if (write_telemetry)
				controlWriter.Write(true_output.ToString("G8") + ',');

            return output;
		}

		[AutoGuiAttr("True output", false, "G8")]
		protected float true_output = 0.0f;

		float far_exponential_blend(float prev, float desire)
		{
			float error = desire - prev;
			if (Math.Abs(error * 20.0f) > 0.1)
			{
				return prev + Common.Clampf(error * TimeWarp.fixedDeltaTime / ferramTimeConstant, Math.Abs(0.6f * error));
			}
			else
				return desire;
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
                        controlWriter = File.CreateText(KSPUtil.ApplicationRootPath + "/Resources/control.csv");
                        v_writer = File.CreateText(KSPUtil.ApplicationRootPath + "/Resources/v.csv");
                        acc_writer = File.CreateText(KSPUtil.ApplicationRootPath + "/Resources/acc.csv");
                        desire_acc_writer = File.CreateText(KSPUtil.ApplicationRootPath + "/Resources/desire.csv");
                        prediction_writer = File.CreateText(KSPUtil.ApplicationRootPath + "/Resources/predict.csv");
						aoa_writer = File.CreateText(KSPUtil.ApplicationRootPath + "/Resources/aoa.csv");
						airspd_writer = File.CreateText(KSPUtil.ApplicationRootPath + "/Resources/airspd.csv");
						density_writer = File.CreateText(KSPUtil.ApplicationRootPath + "/Resources/density.csv");
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
                        desire_acc_writer.Close();
                        prediction_writer.Close();
						aoa_writer.Close();
						airspd_writer.Close();
						density_writer.Close();
                        _write_telemetry = value;
                    }					
				}
			}
		}
		bool _write_telemetry = false;

		#region Parameters

        [AutoGuiAttr("DEBUG desired acc", false, "G8")]
        internal float desired_acc { get; private set; }

		[AutoGuiAttr("FAR csurf time constant", false, "G6")]
		protected float ferramTimeConstant
		{
			get
			{
				return (float)ferram4.FARControllableSurface.timeConstant;
			}
		}

        [AutoGuiAttr("Position", false, "G8")]
        public float coord { get { return vessel.vesselTransform.position[axis]; } }

        [AutoGuiAttr("Velocity", false, "G8")]
        public float vel { get { return vessel.rootPart.rb.velocity[axis]; } }

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
