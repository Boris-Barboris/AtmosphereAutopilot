/*
Copyright 2015, Boris-Barboris

This file is part of Atmosphere Autopilot.
Atmosphere Autopilot is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
Atmosphere Autopilot is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
You should have received a copy of the GNU General Public License
along with Atmosphere Autopilot.  If not, see <http://www.gnu.org/licenses/>. 
*/

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
            AutoTrim = false;
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

        //[AutoGuiAttr("Kp", true, "G8")]
        float Kp = 8.0f;

		/// <summary>
		/// Main control function
		/// </summary>
		/// <param name="cntrl">Control state to change</param>
        public override float ApplyControl(FlightCtrlState cntrl, float target_value)
		{
            vel = imodel.AngularVel(axis);				    // get angular velocity

            float user_input = ControlUtils.get_neutralized_user_input(cntrl, axis);

            if (user_input != 0.0)
            {
                // user is interfering with control
                desired_v = user_input * max_v_construction;
            }
            else
            {
                // control from above
                desired_v = Common.Clampf(target_value, max_v_construction);
            }
            
            if (imodel.dyn_pressure >= 10.0)
                desired_v = moderate_desired_v(desired_v);      // moderation stage

            output_acc = get_desired_acc(desired_v);            // produce output

			// check if we're stable on given input value
            if (AutoTrim)
            {
                if (Math.Abs(vel) < 0.005f)
                {
                    time_in_regime += TimeWarp.fixedDeltaTime;
                }
                else
                {
                    time_in_regime = 0.0;
                }

                if (time_in_regime >= 5.0)
                    ControlUtils.set_trim(axis, imodel.ControlSurfPosHistory(axis).Average());
            }

            acc_controller.ApplyControl(cntrl, output_acc);

            return output_acc;
		}

        [VesselSerializable("max_v_construction")]
        [GlobalSerializable("max_v_construction")]
        [AutoGuiAttr("Max v construction", true, "G8")]
        public float max_v_construction = 0.5f;

        protected virtual float moderate_desired_v(float des_v) { return des_v; }

        protected virtual float get_desired_acc(float des_v) { return Kp * (desired_v - vel); }


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

		[AutoGuiAttr("moder_filter", true, "G6")]
		float moder_filter = 4.0f;

        Matrix state_mat = new Matrix(3, 1);
        Matrix input_mat = new Matrix(1, 1);

        protected override float moderate_desired_v(float des_v)
        {
            float rad_max_aoa = max_aoa * dgr2rad;
            res_max_aoa = 100.0f;
            res_min_aoa = -100.0f;
            res_equilibr_v_upper = max_v_construction;
            res_equilibr_v_lower = -max_v_construction;
            float cur_aoa = Math.Abs(imodel.AoA(axis));

            if (moderate_aoa && imodel.dyn_pressure > 100.0)
            {
                bool stability_region = false;
                res_max_aoa = rad_max_aoa;
                res_min_aoa = -rad_max_aoa;

                if (cur_aoa < 0.26f)
                {
                    // We're in linear regime so we can update our limitations

                    // get equilibrium v for max_aoa
					float new_max_aoa_g =
						-(float)((imodel.pitch_rot_model.C[0, 0] + imodel.pitch_rot_model.A[0, 0] * rad_max_aoa) * vessel.srfSpeed);
					max_aoa_g = (float)Common.simple_filter(new_max_aoa_g, max_aoa_g, moder_filter);
                    max_aoa_v = max_aoa_g / (float)vessel.srfSpeed;
					float new_min_aoa_g = new_max_aoa_g + (float)(2.0 * imodel.pitch_rot_model.A[0, 0] * rad_max_aoa * vessel.srfSpeed);
					min_aoa_g = (float)Common.simple_filter(new_min_aoa_g, min_aoa_g, moder_filter);
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
                            if (!double.IsInfinity(in_eq_x[0, 0]))
                            {
                                if (in_eq_x[0, 0] < 0.0)
                                {
                                    // plane is statically unstable, in_eq_x solution is equilibrium on it's minimal stable aoa
                                    min_input_aoa = (float)Common.simple_filter(0.8 * in_eq_x[0, 0], min_input_aoa, moder_filter);
                                    min_input_v = (float)Common.simple_filter(0.8 * in_eq_x[1, 0], min_input_v, moder_filter);
                                }
                                else
                                {
                                    // plane is statically stable, in_eq_x solution is equilibrium on it's maximal stable aoa
                                    max_input_aoa = (float)Common.simple_filter(in_eq_x[0, 0], max_input_aoa, moder_filter);
                                    max_input_v = (float)Common.simple_filter(in_eq_x[1, 0], max_input_v, moder_filter);
                                }

                                // get equilibrium aoa and angular_v for -1.0 input
                                in_eq_b[1, 0] = imodel.pitch_rot_model.A[1, 2] + imodel.pitch_rot_model.B[1, 0] - imodel.pitch_rot_model.C[1, 0];
                                in_eq_x = in_eq_A.SolveWith(in_eq_b);
                                if (!double.IsInfinity(in_eq_x[0, 0]))
                                {
                                    if (in_eq_x[0, 0] >= 0.0)
                                    {
                                        // plane is statically unstable, in_eq_x solution is equilibrium on it's maximal stable aoa
                                        max_input_aoa = (float)Common.simple_filter(0.8 * in_eq_x[0, 0], max_input_aoa, moder_filter);
                                        max_input_v = (float)Common.simple_filter(0.8 * in_eq_x[1, 0], max_input_v, moder_filter);
                                    }
                                    else
                                    {
                                        // plane is statically stable, in_eq_x solution is equilibrium on it's minimal stable aoa
                                        min_input_aoa = (float)Common.simple_filter(in_eq_x[0, 0], min_input_aoa, moder_filter);
                                        min_input_v = (float)Common.simple_filter(in_eq_x[1, 0], min_input_v, moder_filter);
                                    }
                                    stability_region = true;    // we didn't fail on computing stability region, horay!
                                }
                            }
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

            if (moderate_g && imodel.dyn_pressure > 100.0)
            {
                if (imodel.pitch_rot_model.A[0, 0] != 0.0 && cur_aoa < 0.26)
                {
                    // get equilibrium aoa and angular v for max_g g-force
					max_g_v_upper = (float)Common.simple_filter(
						(max_g_force * 9.81 + imodel.pitch_gravity_acc) / vessel.srfSpeed,
						max_g_v_upper, moder_filter);
					max_g_aoa_upper = (float)Common.simple_filter(
						-(max_g_v_upper + imodel.pitch_rot_model.C[0, 0]) / imodel.pitch_rot_model.A[0, 0],
						max_g_aoa_upper, moder_filter);
					max_g_v_lower = (float)Common.simple_filter(
						(-max_g_force * 9.81 + imodel.pitch_gravity_acc) / vessel.srfSpeed,
						max_g_v_lower, moder_filter);
                    max_g_aoa_lower = (float)Common.simple_filter(
						-(max_g_v_lower + imodel.pitch_rot_model.C[0, 0]) / imodel.pitch_rot_model.A[0, 0],
						max_g_aoa_lower, moder_filter);
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
                        old_dyn_max_v = max_v_construction;
                }
                else
                {
					transit_max_v = (float)Common.simple_filter(new_dyn_max_v / 3.0f, transit_max_v, moder_filter);
                    old_dyn_max_v = transit_max_v;
                }
            }
            else
                if (old_dyn_max_v != 0.0f)
                    transit_max_v = old_dyn_max_v;
                else
                {
                    old_dyn_max_v = max_v_construction;
                    transit_max_v = max_v_construction;
                }

            // desired_v moderation section
            float scaled_restrained_v;
            float normalized_des_v = des_v / max_v_construction;
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

        [AutoGuiAttr("quadr Kp", true, "G6")]
        float quadr_Kp = 0.2f;

        [AutoGuiAttr("kacc_quadr", false, "G6")]
        float kacc_quadr;
        bool first_quadr = true;

        [AutoGuiAttr("kacc_smoothing", true, "G5")]
        float kacc_smoothing = 10.0f;

        [AutoGuiAttr("relaxation_k", true, "G5")]
        float relaxation_k = 1.0f;

        [AutoGuiAttr("relaxation_Kp", true, "G5")]
        float relaxation_Kp = 0.1f;

        [AutoGuiAttr("relaxation_frame", true)]
        int relaxation_frame = 8;

        [AutoGuiAttr("relaxation_frame", false)]
        int relax_count = 0;

        protected override float get_desired_acc(float des_v)
        {
            float new_kacc_quadr = (float)(quadr_Kp * imodel.pitch_rot_model.A[1, 2] * imodel.pitch_rot_model.B[2, 0]);
            if (first_quadr)
                kacc_quadr = new_kacc_quadr;
            else
                kacc_quadr = (float)Common.simple_filter(new_kacc_quadr, kacc_quadr, kacc_smoothing);
            if (kacc_quadr < 1e-3)
                return base.get_desired_acc(des_v);
            first_quadr = false;
            float v_error = vel - des_v;
            double quadr_x;
            float desired_deriv;
            float dt = TimeWarp.fixedDeltaTime;
            if (v_error >= 0.0)
            {
                quadr_x = -Math.Sqrt(v_error / kacc_quadr);
                if (quadr_x >= -relaxation_k * dt)
                {
                    if (++relax_count >= relaxation_frame)
                    {
                        float avg_vel = 0.0f;
                        for (int i = 0; i < relaxation_frame; i++)
                            avg_vel += imodel.AngularVelHistory(PITCH).getFromTail(i);
                        avg_vel /= (float)relaxation_frame;
                        v_error = avg_vel - des_v;
                        if (relax_count > relaxation_frame * 2)
                            relax_count--;
                    }
                    desired_deriv = (float)(relaxation_Kp * v_error / quadr_x);
                }
                else
                {
                    relax_count = 0;
                    desired_deriv = (float)(kacc_quadr * Math.Pow(quadr_x + dt, 2.0) - kacc_quadr * quadr_x * quadr_x) / dt;
                }
            }
            else
            {
                quadr_x = -Math.Sqrt(v_error / -kacc_quadr);
                if (quadr_x >= -relaxation_k * dt)
                {
                    if (++relax_count >= relaxation_frame)
                    {
                        float avg_vel = 0.0f;
                        for (int i = 0; i < relaxation_frame; i++)
                            avg_vel += imodel.AngularVelHistory(PITCH).getFromTail(i);
                        avg_vel /= (float)relaxation_frame;
                        v_error = avg_vel - des_v;
                        if (relax_count > relaxation_frame * 2)
                            relax_count--;
                    }
                    desired_deriv = (float)(relaxation_Kp * v_error / quadr_x);
                }
                else
                {
                    desired_deriv = (float)(-kacc_quadr * Math.Pow(quadr_x + dt, 2.0) + kacc_quadr * quadr_x * quadr_x) / dt;
                    relax_count = 0;
                }
            }            
            return desired_deriv;
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
        float max_g_force = 10.0f;

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

        [AutoGuiAttr("moder_filter", true, "G6")]
        float moder_filter = 4.0f;

        Matrix state_mat = new Matrix(3, 1);
        Matrix input_mat = new Matrix(1, 1);

        protected override float moderate_desired_v(float des_v)
        {
            float cur_aoa = imodel.AoA(YAW);
            // let's get non-overshooting max v value, let's call it transit_max_v
            // we start on 0.0 v and 0.0 yaw aoa and we have stopping_time seconds
            // to accelerate. Resulting speed will be our transit
            if (cur_aoa < 0.26f && imodel.dyn_pressure > 10.0)
            {
                state_mat[0, 0] = 0.0;
                state_mat[2, 0] = 1.0;
                input_mat[0, 0] = 1.0;
                double acc = imodel.roll_rot_model.eval_row(1, state_mat, input_mat);
                float new_dyn_max_v = (float)(stopping_time * Math.Abs(acc));
                transit_max_v = (float)Common.simple_filter(new_dyn_max_v / 3.0f, transit_max_v, moder_filter);
                old_dyn_max_v = transit_max_v;
            }
            else
                if (old_dyn_max_v != 0.0f)
                    transit_max_v = old_dyn_max_v;
                else
                {
                    old_dyn_max_v = max_v_construction;
                    transit_max_v = max_v_construction;
                }

            // desired_v moderation section
            float normalized_des_v = des_v / max_v_construction;
            float scaled_restrained_v = Math.Min(transit_max_v, max_v_construction);
            des_v = normalized_des_v * scaled_restrained_v;

            return des_v;
        }

        [AutoGuiAttr("quadr Kp", true, "G6")]
        float quadr_Kp = 0.2f;

        [AutoGuiAttr("kacc_quadr", false, "G6")]
        float kacc_quadr;
        bool first_quadr = true;

        [AutoGuiAttr("kacc_smoothing", true, "G5")]
        float kacc_smoothing = 10.0f;

        [AutoGuiAttr("relaxation_k", true, "G5")]
        float relaxation_k = 1.0f;

        [AutoGuiAttr("relaxation_Kp", true, "G5")]
        float relaxation_Kp = 0.1f;

        [AutoGuiAttr("relaxation_frame", true)]
        int relaxation_frame = 8;

        [AutoGuiAttr("relaxation_frame", false)]
        int relax_count = 0;

        protected override float get_desired_acc(float des_v)
        {
            float new_kacc_quadr = (float)(quadr_Kp * imodel.roll_rot_model.A[1, 2] * imodel.roll_rot_model.B[2, 0]);
            if (first_quadr)
                kacc_quadr = new_kacc_quadr;
            else
                kacc_quadr = (float)Common.simple_filter(new_kacc_quadr, kacc_quadr, kacc_smoothing);
            if (kacc_quadr < 1e-3)
                return base.get_desired_acc(des_v);
            first_quadr = false;
            float v_error = vel - des_v;
            double quadr_x;
            float desired_deriv;
            float dt = TimeWarp.fixedDeltaTime;
            if (v_error >= 0.0)
            {
                quadr_x = -Math.Sqrt(v_error / kacc_quadr);
                if (quadr_x >= -relaxation_k * dt)
                {
                    if (++relax_count >= relaxation_frame)
                    {
                        float avg_vel = 0.0f;
                        for (int i = 0; i < relaxation_frame; i++)
                            avg_vel += imodel.AngularVelHistory(ROLL).getFromTail(i);
                        avg_vel /= (float)relaxation_frame;
                        v_error = avg_vel - des_v;
                        if (relax_count > relaxation_frame * 2)
                            relax_count--;
                    }
                    desired_deriv = (float)(relaxation_Kp * v_error / quadr_x);
                }
                else
                {
                    relax_count = 0;
                    desired_deriv = (float)(kacc_quadr * Math.Pow(quadr_x + dt, 2.0) - kacc_quadr * quadr_x * quadr_x) / dt;
                }
            }
            else
            {
                quadr_x = -Math.Sqrt(v_error / -kacc_quadr);
                if (quadr_x >= -relaxation_k * dt)
                {
                    if (++relax_count >= relaxation_frame)
                    {
                        float avg_vel = 0.0f;
                        for (int i = 0; i < relaxation_frame; i++)
                            avg_vel += imodel.AngularVelHistory(ROLL).getFromTail(i);
                        avg_vel /= (float)relaxation_frame;
                        v_error = avg_vel - des_v;
                        if (relax_count > relaxation_frame * 2)
                            relax_count--;
                    }
                    desired_deriv = (float)(relaxation_Kp * v_error / quadr_x);
                }
                else
                {
                    desired_deriv = (float)(-kacc_quadr * Math.Pow(quadr_x + dt, 2.0) + kacc_quadr * quadr_x * quadr_x) / dt;
                    relax_count = 0;
                }
            }
            return desired_deriv;
        }

        [AutoGuiAttr("transit_max_v", false, "G6")]
        float transit_max_v;

        float old_dyn_max_v;

        #region ModerationParameters

        [VesselSerializable("stopping_time")]
        [GlobalSerializable("stopping_time")]
        [AutoGuiAttr("stopping_time", true, null)]
        public float stopping_time = 1.0f;

        #endregion
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
