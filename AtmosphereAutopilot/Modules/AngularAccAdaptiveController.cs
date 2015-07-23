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
	public abstract class AngularAccAdaptiveController : SISOController
	{
		protected int axis;

		protected InstantControlModel imodel;

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
		}

		protected override void OnActivate()
		{
            imodel.Activate();
		}

        protected override void OnDeactivate()
        {
			write_telemetry = false;
            imodel.Deactivate();
        }

        [AutoGuiAttr("angular acc", false, "G6")]
        protected float acc;

        //[AutoGuiAttr("model acc", false, "G8")]
        //protected float model_acc;

        [AutoGuiAttr("output", false, "G6")]
        protected float output;

		/// <summary>
		/// Main control function
		/// </summary>
		/// <param name="cntrl">Control state to change</param>
        /// <param name="target_value">Desired angular acceleration</param>
        public override float ApplyControl(FlightCtrlState cntrl, float target_value)
		{
            acc = (float)imodel.AngularAcc(axis);
            //model_acc = (float)imodel.model_acc[axis];
            desired_acc = target_value;

            if (write_telemetry)
            {
                desire_acc_writer.Write(target_value.ToString("G8") + ',');
                acc_writer.Write(acc.ToString("G8") + ',');
                v_writer.Write(imodel.AngularVel(axis).ToString("G8") + ',');
                //prediction_writer.Write(model_acc.ToString("G8") + ',');
				aoa_writer.Write(imodel.AoA(axis).ToString("G8") + ',');
				airspd_writer.Write((imodel.up_srf_v + imodel.fwd_srf_v).magnitude.ToString("G8") + ',');
				density_writer.Write(vessel.atmDensity.ToString("G8") + ',');
            }

            float cur_input_raw = get_required_input(cntrl, desired_acc);
			output = cur_input_raw;

			ControlUtils.set_raw_output(cntrl, axis, output);

            if (write_telemetry)
				controlWriter.Write(csurf_output.ToString("G8") + ',');

            return output;
		}

        protected virtual float get_required_input(FlightCtrlState cntrl, float target_value)
        {
            return ControlUtils.getControlFromState(cntrl, axis);
        }

        [AutoGuiAttr("Csurf output", false, "G6")]
        protected float csurf_output { get { return imodel.ControlSurfPos(axis); } }

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
                        //prediction_writer = File.CreateText(KSPUtil.ApplicationRootPath + "/Resources/predict.csv");
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
                        //prediction_writer.Close();
						aoa_writer.Close();
						airspd_writer.Close();
						density_writer.Close();
                        _write_telemetry = value;
                    }					
				}
			}
		}		
        bool _write_telemetry = false;

        [AutoGuiAttr("DEBUG desired acc", false, "G8")]
        internal float desired_acc { get; private set; }

	}


	//
	// Three realizations
	//

    public sealed class PitchAngularAccController : AngularAccAdaptiveController
    {
        internal PitchAngularAccController(Vessel vessel)
            : base(vessel, "Pitch ang acc controller", 77821329, PITCH)
        { }

        Matrix cur_state = new Matrix(3, 1);
        Matrix input_mat = new Matrix(1, 1);

        [AutoGuiAttr("model_predicted_acc", false, "G6")]
        double model_predicted_acc;

        [AutoGuiAttr("acc_correction", false, "G6")]
        double acc_correction;

        protected override float get_required_input(FlightCtrlState cntrl, float target_value)
        {
            double authority = imodel.pitch_rot_model.B[1, 0];
            // check if we have inadequate model authority
            if (Math.Abs(authority) < 1e-4)
                return cntrl.pitch;

            // get model prediction for next frame
            cur_state[0, 0] = imodel.AoA(PITCH);
            cur_state[1, 0] = imodel.AngularVel(PITCH);
            cur_state[2, 0] = imodel.ControlSurfPos(PITCH);
            input_mat[0, 0] = imodel.ControlInput(PITCH);
            double cur_acc_prediction = imodel.pitch_rot_model.eval_row(1, cur_state, input_mat);

            double cur_model_acc = acc;
            if (imodel.ControlSurfPosHistory(PITCH).Size > 1)
            {
                // get model acceleration for current frame
                cur_state[0, 0] = imodel.AoAHistory(PITCH).getFromTail(1);
                cur_state[1, 0] = imodel.AngularVelHistory(PITCH).getFromTail(1);
                cur_state[2, 0] = imodel.ControlSurfPosHistory(PITCH).getFromTail(1);
                cur_model_acc = imodel.pitch_rot_model.eval_row(1, cur_state, input_mat);
            }
            acc_correction = acc - cur_model_acc;

            double acc_error = target_value - (cur_acc_prediction + acc_correction);
            float new_input = (float)(input_mat[0, 0] + acc_error / authority);
            new_input = Common.Clampf(new_input, 1.0f);
            model_predicted_acc = cur_acc_prediction + acc_correction + authority * (new_input - input_mat[0, 0]);

            return new_input;
        }
    }

	public sealed class RollAngularAccController : AngularAccAdaptiveController
	{
		internal RollAngularAccController(Vessel vessel)
            : base(vessel, "Roll ang acc controller", 77821330, ROLL)
		{ }
	}

	public sealed class YawAngularAccController : AngularAccAdaptiveController
	{
		internal YawAngularAccController(Vessel vessel)
            : base(vessel, "Yaw ang acc controller", 77821331, YAW)
		{ }
	}

}
