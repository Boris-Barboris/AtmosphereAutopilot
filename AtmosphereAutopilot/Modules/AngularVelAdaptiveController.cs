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
        protected float Kp = 0.2f;

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
                if (Math.Abs(vel) < 0.01f)
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
            : base(vessel, "Pitch ang vel controller", 1234444, PITCH)
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

        [AutoGuiAttr("max_input_aoa", false, "G6")]
        float max_input_aoa;

        [AutoGuiAttr("max_input_v", false, "G6")]
        float max_input_v;

        [AutoGuiAttr("min_input_aoa", false, "G6")]
        float min_input_aoa;

        [AutoGuiAttr("min_input_v", false, "G6")]
        float min_input_v;

        [AutoGuiAttr("max_g_aoa_upper", false, "G6")]
        float max_g_aoa_upper;

        [AutoGuiAttr("max_g_aoa_lower", false, "G6")]
        float max_g_aoa_lower;

        [AutoGuiAttr("max_g_v_upper", false, "G6")]
        float max_g_v_upper;

        [AutoGuiAttr("max_g_v_lower", false, "G6")]
        float max_g_v_lower;

        [AutoGuiAttr("max_aoa_g", false, "G6")]
        float max_aoa_g;

        [AutoGuiAttr("max_aoa_v", false, "G6")]
        float max_aoa_v;

        [AutoGuiAttr("min_aoa_g", false, "G6")]
        float min_aoa_g;

        [AutoGuiAttr("min_aoa_v", false, "G6")]
        float min_aoa_v;

        Matrix state_mat = new Matrix(3, 1);
        Matrix input_mat = new Matrix(1, 1);

        protected override float moderate_desired_v(float des_v)
        {
            float rad_max_aoa = max_aoa * dgr2rad;
            res_max_aoa = 100.0f;
            res_min_aoa = -100.0f;
            res_equilibr_v_upper = MaxVConstruction;
            res_equilibr_v_lower = -MaxVConstruction;
            float cur_aoa = Math.Abs(imodel.AoA(axis));

            if (moderate_aoa && imodel.dyn_pressure > 10.0)
            {
                bool stability_region = false;
                res_max_aoa = rad_max_aoa;
                res_min_aoa = -rad_max_aoa;

                if (cur_aoa < 0.26f)
                {
                    // We're in linear regime so we can update our limitations

                    // get equilibrium v for max_aoa
                    max_aoa_g = -(float)((imodel.pitch_rot_model.C[0, 0] + imodel.pitch_rot_model.A[0, 0] * rad_max_aoa) * vessel.srfSpeed);
                    max_aoa_v = max_aoa_g / (float)vessel.srfSpeed;
                    min_aoa_g = max_aoa_g + (float)(2.0 * imodel.pitch_rot_model.A[0, 0] * rad_max_aoa * vessel.srfSpeed);
                    min_aoa_v = min_aoa_g / (float)vessel.srfSpeed;

                    // Check if model is adequate and we have at least some authority
                    if (imodel.pitch_rot_model.B[1, 0] > 1e-4)
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
                            if (in_eq_x[0, 0] < 0.0)
                            {
                                // plane is statically unstable, in_eq_x solution is equilibrium on it's minimal stable aoa
                                min_input_aoa = 0.95f * (float)in_eq_x[0, 0];
                                min_input_v = 0.95f * (float)in_eq_x[1, 0];
                            }
                            else
                            {
                                // plane is statically stable, in_eq_x solution is equilibrium on it's maximal stable aoa
                                max_input_aoa = (float)in_eq_x[0, 0];
                                max_input_v = (float)in_eq_x[1, 0];
                            }

                            // get equilibrium aoa and angular_v for -1.0 input
                            in_eq_b[1, 0] = imodel.pitch_rot_model.A[1, 2] + imodel.pitch_rot_model.B[1, 0] - imodel.pitch_rot_model.C[1, 0];
                            in_eq_x = in_eq_A.SolveWith(in_eq_b);
                            if (in_eq_x[0, 0] >= 0.0)
                            {
                                // plane is statically unstable, in_eq_x solution is equilibrium on it's maximal stable aoa
                                max_input_aoa = 0.95f * (float)in_eq_x[0, 0];
                                max_input_v = 0.95f * (float)in_eq_x[1, 0];
                            }
                            else
                            {
                                // plane is statically stable, in_eq_x solution is equilibrium on it's minimal stable aoa
                                min_input_aoa = (float)in_eq_x[0, 0];
                                min_input_v = (float)in_eq_x[1, 0];
                            }
                            stability_region = true;    // we didn't fail on computing stability region, horay!
                        }
                        catch (MSingularException)
                        {
                            // we won't moderate by stability region
                            //Debug.Log(e.Message + " " + e.StackTrace);
                        }
                    }
                }

                // let's apply moderation with max_aoa equilibrium
                res_equilibr_v_upper = Math.Min(res_equilibr_v_upper, max_aoa_v);
                res_equilibr_v_lower = Math.Max(res_equilibr_v_lower, min_aoa_v);

                if (stability_region)
                {
                    // let's apply moderation with stability region
                    if (max_input_aoa < res_max_aoa)
                    {
                        res_max_aoa = max_input_aoa;
                        res_equilibr_v_upper = Math.Min(max_input_v, res_equilibr_v_upper);
                    }
                    if (min_input_aoa > res_min_aoa)
                    {
                        res_min_aoa = min_input_aoa;
                        res_equilibr_v_lower = Math.Max(min_input_v, res_equilibr_v_lower);
                    }
                }
            }

            if (moderate_g && imodel.dyn_pressure > 10.0)
            {
                if (imodel.pitch_rot_model.A[0, 0] != 0.0 && cur_aoa < 0.26)
                {
                    // get equilibrium aoa and angular v for max_g g-force
                    max_g_v_upper = (max_g_force * 9.81f + (float)imodel.pitch_gravity_acc) / (float)vessel.srfSpeed;
                    max_g_aoa_upper = (float)(-(max_g_v_upper + imodel.pitch_rot_model.C[0, 0]) / imodel.pitch_rot_model.A[0, 0]);
                    max_g_v_lower = (-max_g_force * 9.81f + (float)imodel.pitch_gravity_acc) / (float)vessel.srfSpeed;
                    max_g_aoa_lower = (float)(-(max_g_v_lower + imodel.pitch_rot_model.C[0, 0]) / imodel.pitch_rot_model.A[0, 0]);
                }
                // apply g-force moderation
                if (max_g_aoa_upper < res_max_aoa)
                {
                    res_max_aoa = max_g_aoa_upper;
                    res_equilibr_v_upper = Math.Min(max_g_v_upper, res_equilibr_v_upper);
                }
                if (max_g_aoa_lower > res_min_aoa)
                {
                    res_min_aoa = max_g_aoa_lower;
                    res_equilibr_v_lower = Math.Max(max_g_v_lower, res_equilibr_v_lower);
                }
            }

            // let's get non-overshooting max v value, let's call it transit_max_v
            // we start on 0.0 aoa with transit_max_v and we must not overshoot res_max_aoa
            // while applying -1.0 input all the time
            if (cur_aoa < 0.26f && imodel.dyn_pressure > 10.0)
            {
                double transit_max_aoa = Math.Min(rad_max_aoa, res_max_aoa);
                state_mat[0, 0] = transit_max_aoa / 2.0;
                state_mat[2, 0] = -1.0;
                input_mat[0, 0] = -1.0;
                double acc = imodel.pitch_rot_model.eval_row(1, state_mat, input_mat);
                float new_dyn_max_v =
                    (float)Math.Sqrt(transit_max_aoa * (-acc));
                if (float.IsNaN(new_dyn_max_v))
                {
                    if (old_dyn_max_v != 0.0f)
                        transit_max_v = old_dyn_max_v;
                    else
                        old_dyn_max_v = MaxVConstruction;
                }
                else
                {
                    transit_max_v = new_dyn_max_v / 4.0f;
                    old_dyn_max_v = transit_max_v;
                }
            }
            else
                if (old_dyn_max_v != 0.0f)
                    transit_max_v = old_dyn_max_v;
                else
                {
                    old_dyn_max_v = MaxVConstruction;
                    transit_max_v = MaxVConstruction;
                }

            // desired_v moderation section
            float scaled_restrained_v;
            float normalized_des_v = des_v / MaxVConstruction;
            if (des_v >= 0.0f)
            {
                scaled_aoa = Common.Clampf((res_max_aoa - imodel.AoA(axis)) / 2.0f / res_max_aoa, 1.0f);
                scaled_restrained_v = Math.Min(transit_max_v * normalized_des_v * scaled_aoa + res_equilibr_v_upper * (1.0f - scaled_aoa),
                    transit_max_v * normalized_des_v);
            }
            else
            {
                scaled_aoa = Common.Clampf((res_min_aoa - imodel.AoA(axis)) / 2.0f / res_min_aoa, 1.0f);
                scaled_restrained_v = Math.Max(transit_max_v * normalized_des_v * scaled_aoa + res_equilibr_v_lower * (1.0f - scaled_aoa),
                    transit_max_v * normalized_des_v);
            }
            des_v = scaled_restrained_v;

            return des_v;
        }

        [AutoGuiAttr("transit_max_v", false, "G6")]
        float transit_max_v;
        
        float old_dyn_max_v;

        [AutoGuiAttr("res_max_aoa", false, "G6")]
        public float res_max_aoa;

        [AutoGuiAttr("res_equolibr_v_upper", false, "G6")]
        public float res_equilibr_v_upper;

        [AutoGuiAttr("res_min_aoa", false, "G6")]
        public float res_min_aoa;

        [AutoGuiAttr("res_equolibr_v_lower", false, "G6")]
        public float res_equilibr_v_lower;

        [AutoGuiAttr("DEBUG scaled_aoa", false, "G6")]
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

        [AutoGuiAttr("max G-force", true, "G6")]
        float max_g_force = 12.0f;

        #endregion
    }

	public sealed class RollAngularVelocityController : AngularVelAdaptiveController
	{
		internal RollAngularVelocityController(Vessel vessel)
            : base(vessel, "Roll ang vel controller", 1234445, ROLL)
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
            : base(vessel, "Yaw ang vel controller", 1234446, YAW)
		{ }

		public override void InitializeDependencies(Dictionary<Type, AutopilotModule> modules)
		{
			base.InitializeDependencies(modules);
			this.acc_controller = modules[typeof(YawAngularAccController)] as YawAngularAccController;
		}
	}

}
