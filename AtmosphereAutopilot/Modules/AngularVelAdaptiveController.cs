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
	public abstract class AngularVelAdaptiveController : SIMOController
	{
		protected int axis;

        protected InstantControlModel model;
        protected MediumFlightModel mmodel;
		protected AngularAccAdaptiveController acc_controller;


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
		protected AngularVelAdaptiveController(Vessel vessel, string module_name,
			int wnd_id, int axis)
			: base(vessel, module_name, wnd_id)
		{
			this.axis = axis;
            AutoTrim = true;
		}

		public override void InitializeDependencies(Dictionary<Type, AutopilotModule> modules)
		{
			this.model = modules[typeof(InstantControlModel)] as InstantControlModel;
			this.mmodel = modules[typeof(MediumFlightModel)] as MediumFlightModel;
		}

		protected override void OnActivate() 
        {
            model.Activate();
            mmodel.Activate();
            acc_controller.Activate();
        }

        protected override void OnDeactivate()
        {
            model.Deactivate();
            mmodel.Deactivate();
            acc_controller.Deactivate();
        }

		double time_in_regime = 0.0;

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

            double user_input = ControlUtils.get_neutralized_user_input(cntrl, axis);
            if (user_input != 0.0)
                desired_v = fbw_v_k * user_input * mmodel.max_angular_v[axis];      // user is interfering with control
            else
                desired_v = target_value;                                           // control from above
            
            desired_v = moderate_desired_v(desired_v);      // moderation stage

            output = Common.Clamp(pid.Control(input, desired_v), fbw_dv_k * mmodel.max_angular_dv[axis]);

            error = desired_v - input;
            proport = error * pid.KP;

            acc_controller.ApplyControl(cntrl, output);

			// check if we're stable on given input value
            if (AutoTrim)
            {
                if (Math.Abs(input) < 5e-3)
                {
                    time_in_regime += TimeWarp.fixedDeltaTime;
                }
                else
                {
                    time_in_regime = 0.0;
                }

                if (time_in_regime >= 5.0)
                    ControlUtils.set_trim(axis, model);
            }

            return output;
		}

        protected virtual double moderate_desired_v(double des_v) { return des_v; }


		#region Parameters

		[GlobalSerializable("fbw_v_k")]
        [VesselSerializable("fbw_v_k")]
        [AutoGuiAttr("moderation v k", true, "G6")]
        protected double fbw_v_k = 1.0;

        [GlobalSerializable("fbw_dv_k")]
        [VesselSerializable("fbw_dv_k")]
        [AutoGuiAttr("moderation dv k", true, "G6")]
        protected double fbw_dv_k = 1.0;

        [AutoGuiAttr("DEBUG proport", false, "G8")]
        internal double proport { get; private set; }

        [AutoGuiAttr("DEBUG desired_v", false, "G8")]
        protected double desired_v;

		[GlobalSerializable("kp_acc_factor")]
		[AutoGuiAttr("KP acceleration factor", true, "G6")]
		protected double kp_acc_factor = 0.5;

        [GlobalSerializable("AutoTrim")]
        [AutoGuiAttr("AutoTrim", true, null)]
        public bool AutoTrim { get; set; }

		#endregion

	}


	//
	// Three realizations
	//

    public sealed class PitchAngularVelocityController : AngularVelAdaptiveController
    {
        internal PitchAngularVelocityController(Vessel vessel)
            : base(vessel, "Adaptive elavator trimmer", 1234444, PITCH)
        { }

		public override void InitializeDependencies(Dictionary<Type, AutopilotModule> modules)
		{
			base.InitializeDependencies(modules);
			this.acc_controller = modules[typeof(PitchAngularAccController)] as PitchAngularAccController;
		}

        protected override double moderate_desired_v(double des_v)
        {
            // limit it due to g-force limitations
            double cur_g = mmodel.g_force.getLast();
            if (des_v * mmodel.aoa_pitch.getLast() > 0.0)
            {
                // user is trying to increase AoA
                max_g = fbw_g_k / mmodel.wing_load_k / mmodel.wing_load_k;
                double g_relation = 1.0;
                double aoa_relation = 1.0;
                if (moderate_g)
                    if (max_g > 1e-3 && cur_g >= 0.0)
                    {
                        double stasis_angular_spd = max_g / vessel.srfSpeed;
                        double k = 1.0 - Common.Clamp(stasis_angular_spd / des_v, 0.0, 1.0);
                        g_relation = k * cur_g / max_g;
                    }
                if (moderate_aoa)
                    if (fbw_max_aoa > 2.0)
                    {
                        const double dgr_to_rad = 1.0 / 180.0 * Math.PI;
                        double max_aoa_rad = fbw_max_aoa * dgr_to_rad;
                        aoa_relation = Math.Abs(mmodel.aoa_pitch.getLast()) / max_aoa_rad;
                    }
                double max_k = Math.Max(aoa_relation, g_relation);
                fbw_modifier = Common.Clamp(1.0 - max_k, 1.0);
                des_v *= fbw_modifier;
            }
            return des_v;
        }

        #region ModerationParameters

        [GlobalSerializable("moderate_aoa")]
        [VesselSerializable("moderate_aoa")]
        [AutoGuiAttr("Moderate AoA", true, null)]
        public bool moderate_aoa = true;

        [GlobalSerializable("moderate_g")]
        [VesselSerializable("moderate_g")]
        [AutoGuiAttr("Moderate G-force", true, null)]
        public bool moderate_g = true;

        [GlobalSerializable("fbw_g_k")]
        [VesselSerializable("fbw_g_k")]
        [AutoGuiAttr("max g-force k", true, "G6")]
        double fbw_g_k = 1.0;

        [GlobalSerializable("fbw_max_aoa")]
        [VesselSerializable("fbw_max_aoa")]
        [AutoGuiAttr("max AoA degrees", true, "G6")]
        double fbw_max_aoa = 15.0;

        [AutoGuiAttr("DEBUG g_fwb_modifier", false, "G8")]
        double fbw_modifier;

        [AutoGuiAttr("DEBUG max_g", false, "G8")]
        double max_g;

        #endregion
    }

	public sealed class RollAngularVelocityController : AngularVelAdaptiveController
	{
		internal RollAngularVelocityController(Vessel vessel)
			: base(vessel, "Adaptive roll trimmer", 1234445, ROLL)
		{ }

		public override void InitializeDependencies(Dictionary<Type, AutopilotModule> modules)
		{
			base.InitializeDependencies(modules);
			this.acc_controller = modules[typeof(RollAngularAccController)] as RollAngularAccController;
		}
	}

	public sealed class YawAngularVelocityController : AngularVelAdaptiveController
	{
		internal YawAngularVelocityController(Vessel vessel)
			: base(vessel, "Adaptive yaw trimmer", 1234446, YAW)
		{ }

		public override void InitializeDependencies(Dictionary<Type, AutopilotModule> modules)
		{
			base.InitializeDependencies(modules);
			this.acc_controller = modules[typeof(YawAngularAccController)] as YawAngularAccController;
		}
	}

}
