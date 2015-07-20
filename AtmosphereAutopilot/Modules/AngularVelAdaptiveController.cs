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
	public abstract class AngularVelAdaptiveController : SISOController
	{
		protected int axis;

        protected InstantControlModel imodel;
		protected AngularAccAdaptiveController acc_controller;

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
			this.imodel = modules[typeof(InstantControlModel)] as InstantControlModel;
		}

		protected override void OnActivate() 
        {
            imodel.Activate();
            acc_controller.Activate();
        }

        protected override void OnDeactivate()
        {
            imodel.Deactivate();
            acc_controller.Deactivate();
        }

		double time_in_regime = 0.0;

        [AutoGuiAttr("angular vel", false, "G8")]
        protected float vel;

        [AutoGuiAttr("output acceleration", false, "G8")]
        protected float output_acc;

        [AutoGuiAttr("Kp", true, "G8")]
        protected float Kp = 1.0f;

		/// <summary>
		/// Main control function
		/// </summary>
		/// <param name="cntrl">Control state to change</param>
        public override float ApplyControl(FlightCtrlState cntrl, float target_value)
		{
            vel = imodel.AngularVel(axis);				    // get angular velocity

            float user_input = ControlUtils.get_neutralized_user_input(cntrl, axis);

            // Moderate desired v by constructional limits
            // max_v_construction = TODO
            MaxVConstruction = max_v_construction * max_v_construction_k;

            if (user_input != 0.0)
            {
                // user is interfering with control
                desired_v = user_input * MaxVConstruction;
            }
            else
            {
                // control from above
                desired_v = Common.Clampf(target_value, MaxVConstruction);
            }
            
            if (imodel.dyn_pressure >= 10.0)
                desired_v = moderate_desired_v(desired_v);      // moderation stage

            output_acc = Kp * (desired_v - vel) / TimeWarp.fixedDeltaTime;            // produce output

			// check if we're stable on given input value
            if (AutoTrim)
            {
                if (Math.Abs(vel) < 5e-3f)
                {
                    time_in_regime += TimeWarp.fixedDeltaTime;
                }
                else
                {
                    time_in_regime = 0.0;
                }

                if (time_in_regime >= 3.0)
                    ControlUtils.set_trim(cntrl, axis, imodel);
            }

            acc_controller.ApplyControl(cntrl, output_acc);

            return output_acc;
		}

        protected float max_v_construction = 0.5f;

        [VesselSerializable("max_v_construction_k")]
        [AutoGuiAttr("Max v construct mult", true, "G8")]
        protected float max_v_construction_k = 1.0f;

        [AutoGuiAttr("Max V construct", false, "G8")]
        public float MaxVConstruction { get; private set; }

        protected virtual float moderate_desired_v(float des_v) { return des_v; }


		#region Parameters

        [AutoGuiAttr("DEBUG desired_v", false, "G8")]
        protected float desired_v;

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

        Matrix in_eq_A = new Matrix(2, 2);
        Matrix in_eq_b = new Matrix(2, 1);
        Matrix in_eq_x;

        const float dgr2rad = (float)(Math.PI / 180.0);

        [AutoGuiAttr("controllable_aoa", false, "G8")]
        float controllable_aoa;

        [AutoGuiAttr("controllable_v", false, "G8")]
        float controllable_v;

        [AutoGuiAttr("max_g_aoa", false, "G8")]
        float max_g_aoa;

        [AutoGuiAttr("max_g_v", false, "G8")]
        float max_g_v;

        [AutoGuiAttr("max_aoa_g", false, "G8")]
        float max_aoa_g;

        [AutoGuiAttr("max_aoa_v", false, "G8")]
        float max_aoa_v;

        protected override float moderate_desired_v(float des_v)
        {
            res_max_aoa = max_aoa * dgr2rad;
            equilibr_max_v = MaxVConstruction;
            float aoa = Math.Abs(imodel.AoA(axis));
            if (moderate_aoa)
            {
                if (aoa < 0.26f)
                {
                    // get equilibrium aoa and angular_v for 1.0 input
                    in_eq_A[0, 0] = imodel.pitch_rot_model.A[0, 0];
                    in_eq_A[0, 1] = imodel.pitch_rot_model.A[0, 1];
                    in_eq_A[1, 0] = imodel.pitch_rot_model.A[1, 0];
                    in_eq_b[0, 0] = -imodel.pitch_rot_model.C[0, 0];
                    in_eq_b[1, 0] = -imodel.pitch_rot_model.A[1, 2] - imodel.pitch_rot_model.B[1, 0] - imodel.pitch_rot_model.C[1, 0];
                    in_eq_A.old_lu = true;
                    try
                    {
                        in_eq_x = in_eq_A.SolveWith(in_eq_b);
                        if (in_eq_x != null)
                        {
                            res_max_aoa = Common.Clampf(res_max_aoa, (float)in_eq_x[0, 0]);
                            equilibr_max_v = Common.Clampf(equilibr_max_v, (float)in_eq_x[1, 0]);
                            controllable_aoa = Math.Abs((float)in_eq_x[0, 0]);
                            controllable_v = Math.Abs((float)in_eq_x[1, 0]);
                        }
                    }
                    catch (MSingularException e)
                    {
                        // we won't try to moderate
                        Debug.Log(e.Message + " " + e.StackTrace);
                    }
                    
                    // get equilibrium v for max_aoa aoa
                    max_aoa_g = (float)(imodel.pitch_coeffs.Cl0 + imodel.pitch_coeffs.Cl1 * max_aoa * dgr2rad + imodel.pitch_gravity_acc);
                    max_aoa_v = max_aoa_g / (float)vessel.srfSpeed;
                }                
                equilibr_max_v = Math.Min(equilibr_max_v, max_aoa_v);
            }
            if (moderate_g)
            {
                // get equilibrium aoa and angular v for max_g g-force
                if (imodel.pitch_coeffs.Cl1 != 0.0)
                {
                    max_g_v = (max_g * 9.81f + (float)imodel.pitch_gravity_acc) / (float)vessel.srfSpeed;
                    max_g_aoa = (float)((max_g * 9.81f - imodel.pitch_coeffs.Cl0) / imodel.pitch_coeffs.Cl1);
                    res_max_aoa = Common.Clampf(res_max_aoa, max_g_aoa);
                    equilibr_max_v = Math.Min(equilibr_max_v, max_g_v);
                }                
            }

            // now get dynamical non-overshooting max v value
            transit_max_v = MaxVConstruction;
            if (aoa < 0.26f)
            {
                float new_dyn_max_v = Math.Min(
                    (float)Math.Sqrt(res_max_aoa * (imodel.pitch_coeffs.k0 +
                    imodel.pitch_coeffs.k1 * res_max_aoa / 2.0 + imodel.pitch_coeffs.k2 +
                    imodel.reaction_torque[PITCH] / imodel.MOI[PITCH])),
                    MaxVConstruction);
                if (float.IsNaN(new_dyn_max_v))
                {
                    transit_max_v = old_dyn_max_v;
                    Debug.Log("NaN in new_dyn_max_v, res_max_aoa = " + res_max_aoa.ToString("G8") +
                        "; imodel.pitch_coeffs.k0 = " + imodel.pitch_coeffs.k0.ToString("G8") +
                        "; imodel.pitch_coeffs.k1 = " + imodel.pitch_coeffs.k1.ToString("G8") +
                        "; imodel.pitch_coeffs.k2 = " + imodel.pitch_coeffs.k2.ToString("G8") + 
                        "; imodel.reaction_torque[PITCH] / imodel.MOI[PITCH] = " + 
                        (imodel.reaction_torque[PITCH] / imodel.MOI[PITCH]).ToString("G8"));
                }
                else
                {
                    transit_max_v = new_dyn_max_v;
                    old_dyn_max_v = transit_max_v;
                }
            }
            else
                if (old_dyn_max_v != 0.0f)
                    transit_max_v = old_dyn_max_v;
                else
                    old_dyn_max_v = MaxVConstruction;

            // Now saturate desired_v if needed
            if ((moderate_g || moderate_aoa) && res_max_aoa != 0.0f)
            {
                float scaled_restrained_v;
                float normalized_des_v = des_v / MaxVConstruction;
                if (des_v >= 0.0f)
                {
                    scaled_aoa = (res_max_aoa - imodel.AoA(axis)) / 2.0f / res_max_aoa;
                    scaled_restrained_v = Math.Min(transit_max_v * normalized_des_v * scaled_aoa + equilibr_max_v * (1.0f - scaled_aoa),
                        transit_max_v * normalized_des_v);
                }
                else
                {
                    scaled_aoa = (res_max_aoa + imodel.AoA(axis)) / 2.0f / res_max_aoa;
                    scaled_restrained_v = Math.Max(transit_max_v * normalized_des_v * scaled_aoa - equilibr_max_v * (1.0f - scaled_aoa),
                        transit_max_v * normalized_des_v);
                }
                des_v = scaled_restrained_v;
            }
            return des_v;
        }

        [AutoGuiAttr("transit_max_v", false, "G8")]
        float transit_max_v;
        
        float old_dyn_max_v;

        [AutoGuiAttr("model max aoa", false, "G8")]
        public float res_max_aoa;

        [AutoGuiAttr("equilibrium max v", false, "G8")]
        public float equilibr_max_v;

        [AutoGuiAttr("DEBUG scaled_aoa", false, "G8")]
        float scaled_aoa;

        #region ModerationParameters

        [VesselSerializable("moderate_aoa")]
        [AutoGuiAttr("Moderate AoA", true, null)]
        public bool moderate_aoa = true;

        [VesselSerializable("moderate_g")]
        [AutoGuiAttr("Moderate G-force", true, null)]
        public bool moderate_g = true;

        [VesselSerializable("max_aoa")]
        [AutoGuiAttr("max AoA", true, "G6")]
        float max_aoa = 15.0f;

        [AutoGuiAttr("max G-force", true, "G8")]
        float max_g = 12.0f;

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
