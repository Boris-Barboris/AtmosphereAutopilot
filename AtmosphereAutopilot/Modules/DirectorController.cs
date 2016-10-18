/*
Atmosphere Autopilot, plugin for Kerbal Space Program.
Copyright (C) 2015-2016, Baranin Alexander aka Boris-Barboris.
 
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
    /// Controls surface velocity vector
    /// </summary>
    public sealed class DirectorController : AutopilotModule
    {
        internal DirectorController(Vessel v)
            : base(v, 88443289, "Director controller")
        { }

        PitchAoAController aoa_c;
        SideslipController side_c;
        PitchAngularVelocityController pitch_c;
        YawAngularVelocityController yaw_c;
        RollAngularVelocityController roll_c;
        FlightModel imodel;

        public override void InitializeDependencies(Dictionary<Type, AutopilotModule> modules)
        {
            imodel = modules[typeof(FlightModel)] as FlightModel;
            aoa_c = modules[typeof(PitchAoAController)] as PitchAoAController;
            side_c = modules[typeof(SideslipController)] as SideslipController;
            pitch_c = modules[typeof(PitchAngularVelocityController)] as PitchAngularVelocityController;
            roll_c = modules[typeof(RollAngularVelocityController)] as RollAngularVelocityController;
            yaw_c = modules[typeof(YawAngularVelocityController)] as YawAngularVelocityController;
        }

        bool aoa_moderation_saved = false;
        bool sideslip_moderation_saved = false;

        protected override void OnActivate()
        {
            imodel.Activate();
            aoa_c.Activate();
            yaw_c.Activate();
            roll_c.Activate();
            // save moderation states
            aoa_moderation_saved = pitch_c.moderate_aoa;
            sideslip_moderation_saved = yaw_c.moderate_aoa;
        }

        protected override void OnDeactivate()
        {
            imodel.Deactivate();
            aoa_c.Deactivate();
            yaw_c.Deactivate();
            roll_c.Deactivate();
            // restore moderation states
            pitch_c.moderate_aoa = aoa_moderation_saved;
            yaw_c.moderate_aoa = sideslip_moderation_saved;
        }

        [AutoGuiAttr("strength", true, "G4")]
        public double strength = 0.95;

        [AutoGuiAttr("roll_stop_k", true, "G5")]
        public float roll_stop_k = 1.0f;

        [AutoGuiAttr("angular error", false, "G5")]
        public double angular_error;

        [AutoGuiAttr("max_angular_v", false, "G5")]
        public double max_angular_v;

        [AutoGuiAttr("stop_time_roll", false, "G5")]
        public double stop_time_roll;

        [AutoGuiAttr("relaxation_margin", true, "G5")]
        public double relaxation_margin = 0.01;

        [AutoGuiAttr("angle_relaxation_k", true, "G5")]
        public float angle_relaxation_k = 0.1f;

        [VesselSerializable("max_neg_g")]
        [AutoGuiAttr("max_neg_g", true, "G5")]
        public double max_neg_g = 8.0;

        [AutoGuiAttr("min_rollover_alt", true, "G5")]
        public double min_rollover_alt = 150.0;

        public double max_lift_acc = 0.0;
        public double max_sideslip_acc = 0.0;

        /// <summary>
        /// Main control function
        /// </summary>
        /// <param name="desired_vel">Desired velocity direction in surface reference frame.</param>
        /// <param name="desired_acceleration">Desired acceleration.</param>
        public void ApplyControl(FlightCtrlState state, Vector3d desired_vel, Vector3d desired_acceleration)
        {
            Vector3d planet2ves = vessel.ReferenceTransform.position - vessel.mainBody.position;
            Vector3d planet2vesNorm = planet2ves.normalized;
            Vector3d shift_acc = Vector3d.zero;

            // centrifugal acceleration to stay on desired altitude
            //Vector3d level_acc = -planet2vesNorm * (imodel.surface_v - Vector3d.Project(imodel.surface_v, planet2vesNorm)).sqrMagnitude / planet2ves.magnitude;

            // Rotation vector
            Vector3d desired_turn_acc_dir = Vector3d.Cross(
                Vector3d.Cross(imodel.surface_v, desired_vel).normalized,
                imodel.surface_v).normalized;
            angular_error = Vector3d.Angle(imodel.surface_v.normalized, desired_vel) * dgr2rad;

            max_lift_acc = Math.Max(0.01, max_lift_acceleration(PITCH));
            max_sideslip_acc = Math.Max(0.001, max_lift_acceleration(YAW));

            // let's find this craft's maximum acceleration toward desired_turn_acc_dir without stalling
            // it can be solved from simple quadratic equation
            Vector3d max_turn_acc = non_stall_turn_acceleration(desired_turn_acc_dir, max_lift_acc);
            max_turn_acc = strength * max_turn_acc * ((vessel == FlightGlobals.ActiveVessel && FlightInputHandler.fetch.precisionMode) ? 0.4 : 1.0);

            // now let's take roll speed and relaxation into account
            double max_angular_v = max_turn_acc.magnitude / imodel.surface_v_magnitude;
            double t1 = max_roll_v / roll_acc_factor;
            double t2 = (90.0 * dgr2rad - t1 * max_roll_v) / max_roll_v;
            double stop_time_roll = roll_stop_k * (2.0 * t1 + t2);
            if (double.IsNaN(stop_time_roll) || double.IsInfinity(stop_time_roll))
                stop_time_roll = 2.0;
            if (stop_time_roll <= 0.0)
                stop_time_roll = 0.5;

            // now let's generate desired acceleration
            if (angular_error / max_angular_v > stop_time_roll)
            {
                // we're far away from relaxation, let's simply produce maximum acceleration
                shift_acc = max_turn_acc;
            }
            else
            {
                // we're relaxing now, quadratic descend is good approximation
                {
                    double tk = (angular_error / max_angular_v) / stop_time_roll;
                    if (Math.Abs(angular_error) < relaxation_margin)
                        tk *= Mathf.Lerp(1.0f, angle_relaxation_k, (float)(1.0f - Math.Abs(angular_error) / relaxation_margin));
                    shift_acc = max_turn_acc * tk;
                }
            }

            //if (angular_error > 0.2 || Vector3d.Dot(desired_acceleration, shift_acc) < -0.1)
            //    target_acc = shift_acc;
            //else
            target_acc = desired_acceleration + shift_acc;
            //current_acc = imodel.sum_acc;

            // we need aoa moderation for AoA controllers to work
            pitch_c.moderate_aoa = true;
            yaw_c.moderate_aoa = true;

            Vector3d neutral_acc = -imodel.gravity_acc - imodel.noninert_acc;
            Vector3d target_lift_acc = target_acc + neutral_acc;
            Vector3d target_normal_lift_acc = target_lift_acc - Vector3d.Project(target_lift_acc, imodel.surface_v);

            // prevent rolling on small errors
            if (angular_error < 2e-2 && target_normal_lift_acc.magnitude < neutral_acc.magnitude * 0.3)
            {
                target_lift_acc = Vector3d.Project(target_lift_acc, neutral_acc);
                target_normal_lift_acc = target_lift_acc - Vector3d.Project(target_lift_acc, imodel.surface_v);
            }

            Vector3d desired_right_direction = Vector3d.Cross(target_normal_lift_acc, vessel.ReferenceTransform.up).normalized;

            // let's apply roll to maintain desired_right_direction
            Vector3 right_vector = imodel.virtualRotation * Vector3.right;
            double new_roll_error = Math.Sign(Vector3d.Dot(right_vector, target_normal_lift_acc)) *
                Math.Acos(Math.Min(Math.Max(Vector3d.Dot(desired_right_direction, right_vector), -1.0), 1.0));
            // rolling to pitch up is not always as efficient as pitching down
            double spine_to_zenith = Vector3d.Dot(desired_right_direction, Vector3d.Cross(imodel.surface_v, imodel.gravity_acc));
            if (target_normal_lift_acc.magnitude < max_neg_g * 9.81 || !allow_spine_down || vessel.heightFromTerrain < min_rollover_alt)
                if (Math.Abs(new_roll_error) > 90.0 * dgr2rad && spine_to_zenith < 0.0)
                    new_roll_error = new_roll_error - 180.0 * dgr2rad * Math.Sign(new_roll_error);
            // filter it
            if ((Math.Abs(new_roll_error) < roll_error_filter_margin * dgr2rad) && (Math.Abs(roll_error) < roll_error_filter_margin * dgr2rad))
                roll_error = Common.simple_filter(new_roll_error, roll_error, roll_error_filter_k);
            else
                roll_error = new_roll_error;
            // generate desired roll angular_v
            roll_c.user_controlled = false;
            roll_c.ApplyControl(state, get_desired_roll_v(roll_error));

            // now let's apply pitch and yaw AoA controls

            // pitch AoA

            desired_pitch_lift = 0.0;
            if (Math.Abs(roll_error) < 30.0 * dgr2rad || Math.Abs(roll_error - 180.0) < 30.0 * dgr2rad)
                desired_pitch_lift = Vector3.Dot(imodel.pitch_tangent, target_normal_lift_acc);
            else
                desired_pitch_lift = Vector3.Dot(imodel.pitch_tangent, neutral_acc);
            desired_pitch_acc = desired_pitch_lift + imodel.pitch_gravity_acc + imodel.pitch_noninert_acc;
            desired_pitch_v = desired_pitch_acc / imodel.surface_v_magnitude;
            // let's find equilibrium AoA for desired lift
            desired_aoa = get_desired_aoa(imodel.pitch_rot_model_gen, desired_pitch_v, 0.0);
            if (float.IsNaN(desired_aoa) || float.IsInfinity(desired_aoa))
                desired_aoa = 0.0f;
            aoa_c.user_controlled = false;
            aoa_c.ApplyControl(state, desired_aoa, 0.0f);

            // yaw sideslip

            //if (Math.Abs(roll_angle) > 3.0 * dgr2rad)
            //{
            //    desired_yaw_lift = 0.0;
            //    desired_sideslip = 0.0f;
            //}
            //else
            //{
            desired_yaw_lift = 0.0; // Vector3.Dot(imodel.yaw_tangent, normal_lift_acc);
            desired_yaw_acc = desired_yaw_lift + imodel.yaw_gravity_acc + imodel.yaw_noninert_acc;
            desired_yaw_v = desired_yaw_acc / imodel.surface_v_magnitude;
            // let's find equilibrium sideslip for desired lift
            //if (Math.Abs(desired_yaw_lift) < 0.01f)
            desired_sideslip = (float)Common.simple_filter(get_desired_aoa(imodel.yaw_rot_model_gen, desired_yaw_v, 0.0), desired_sideslip, sideslip_filter_k);
            if (float.IsNaN(desired_sideslip) || float.IsInfinity(desired_sideslip) || Math.Abs(desired_sideslip) < 0.001f)
                desired_sideslip = 0.0f;
            desired_sideslip = 0.0f;
            //}
            //desired_sideslip = 0.0f;
            side_c.user_controlled = false;
            side_c.ApplyControl(state, desired_sideslip, 0.0f);
        }

        //[AutoGuiAttr("target_acc", false, "G3")]
        public Vector3d target_acc;

        //[AutoGuiAttr("current_acc", false, "G3")]
        public Vector3d current_acc;

        //[AutoGuiAttr("desired_yaw_lift", false, "G5")]
        public double desired_yaw_lift;

        //[AutoGuiAttr("desired_yaw_acc", false, "G5")]
        public double desired_yaw_acc;

        //[AutoGuiAttr("desired_yaw_v", false, "G5")]
        public double desired_yaw_v;

        //[AutoGuiAttr("sideslip_filter_k", true, "G4")]
        public double sideslip_filter_k = 10.0;

        [AutoGuiAttr("desired_pitch_lift", false, "G5")]
        public double desired_pitch_lift;

        [AutoGuiAttr("desired_pitch_acc", false, "G5")]
        public double desired_pitch_acc;

        [AutoGuiAttr("desired_pitch_v", false, "G5")]
        public double desired_pitch_v;

        /// <summary>
        /// Allow rotating belly-up to prevent too large negative g-forces
        /// </summary>
        [AutoGuiAttr("allow_spine_down", true)]
        public bool allow_spine_down = true;

        public Vector3d non_stall_turn_acceleration(Vector3d desired_turn_acc_dir, double max_lift_acc)
        {
            Vector3d max_turn_acc = Vector3d.zero;
            Vector3d rest_lift = imodel.pitch_tangent * (-imodel.pitch_gravity_acc - imodel.pitch_noninert_acc);
            double b = 2.0 * Vector3d.Dot(rest_lift, desired_turn_acc_dir);
            double a = desired_turn_acc_dir.sqrMagnitude;
            double c = rest_lift.sqrMagnitude - max_lift_acc * max_lift_acc;
            if (Math.Abs(a) > 1e-10)
            {
                double discriminant = b * b - 4.0 * a * c;
                if (discriminant < 0.0)
                {
                    // we'll stall anyways, so let's just do a max-force turn
                    max_turn_acc = max_lift_acc * desired_turn_acc_dir;
                }
                else
                {
                    double disc_root = Math.Sqrt(discriminant);
                    double k = (-b + disc_root) / 2.0 / a;
                    max_turn_acc = desired_turn_acc_dir * k;
                }
            }
            else
            {
                if (Math.Abs(b) > 1e-10)
                {
                    double k = -c / b;
                    max_turn_acc = desired_turn_acc_dir * k;
                }
                else
                    max_turn_acc = max_lift_acc * desired_turn_acc_dir;
            }
            return max_turn_acc;
        }

        # region Roll

        Matrix roll_state_m = new Matrix(3, 1);
        Matrix roll_input_m = new Matrix(3, 1);

        /// <summary>
        /// Max roll angular acceleration
        /// </summary>
        [AutoGuiAttr("roll_acc_factor", false, "G5")]
        public double roll_acc_factor = 0.0;

        [AutoGuiAttr("roll_acc_filter", true, "G5")]
        public double roll_acc_filter = 4.0;

        [AutoGuiAttr("roll_cubic_K", true, "G5")]
        public double roll_cubic_K = 0.3;

        [AutoGuiAttr("roll_cubic_relax_frame", true, "G5")]
        public double roll_relax_frame = 10.0;

        [AutoGuiAttr("roll_relax_Kp", true, "G5")]
        public double roll_relax_Kp = 0.1;

        [AutoGuiAttr("roll_error_filter_margin", true, "G5")]
        public double roll_error_filter_margin = 3.0;

        [AutoGuiAttr("roll_error_filter_k", true, "G5")]
        public double roll_error_filter_k = 0.5;

        /// <summary>
        /// Max roll angular velocity
        /// </summary>
        [AutoGuiAttr("max_roll_v", false, "G5")]
        public double max_roll_v;

        [AutoGuiAttr("roll_error", false, "G5")]
        public double roll_error = 0.0;

        [AutoGuiAttr("roll_cubic", false)]
        public bool cubic;

        [AutoGuiAttr("roll_snap_boundary", true, "G5")]
        public double snapping_boundary = 3.0;

        /// <summary>
        /// Calculate input for roll velocity controller
        /// </summary>
        /// <param name="angle_error">Roll error</param>
        /// <returns>desired roll angular velocity</returns>
        public float get_desired_roll_v(double angle_error)
        {
            roll_state_m[0, 0] = 0.0;
            roll_state_m[1, 0] = 1.0;
            roll_state_m[2, 0] = 1.0;
            roll_input_m[0, 0] = 1.0;
            double roll_max_acc = imodel.roll_rot_model_gen.eval_row(0, roll_state_m, roll_input_m) * ((vessel == FlightGlobals.ActiveVessel && FlightInputHandler.fetch.precisionMode) ? 0.4 : 1.0) * strength;
            roll_acc_factor = Common.simple_filter(Math.Abs(roll_max_acc), roll_acc_factor, roll_acc_filter);

            // calculate anti-overshooting perameters
            if (angle_error >= 0.0)
                max_roll_v = Math.Min(roll_c.max_input_v, roll_c.max_v_construction);
            else
                max_roll_v = Math.Abs(Math.Max(roll_c.min_input_v, -roll_c.max_v_construction));
            max_roll_v *= strength;
            double stop_time = Math.Max(max_roll_v / Math.Max(1e-3, roll_acc_factor) + 1.0f / SyncModuleControlSurface.CSURF_SPD, 0.1);

            // we'll use cubic descend to desired bank angle
            double cubic_k = roll_cubic_K * Math.Min(roll_acc_factor / 6.0, max_roll_v / 3.0 / stop_time / stop_time);
            double stop_angle_frame = Math.Sqrt(max_roll_v / 3.0 / cubic_k);

            if (angle_error > stop_angle_frame)
            {
                cubic = false;
                return (float)max_roll_v * Math.Sign(angle_error);
            }
            else
            {
                // we're in cubic section
                double snap_bound = snapping_boundary * dgr2rad;
                if (Math.Abs(angle_error) < snap_bound)
                {
                    // error is too small, let's use snapping code
                    double new_dyn_max_v = Math.Sqrt(snap_bound * roll_acc_factor);
                    if (!double.IsNaN(new_dyn_max_v))
                    {
                        new_dyn_max_v = Common.Clamp(new_dyn_max_v, max_roll_v);
                        return (float)(roll_c.snapping_Kp * angle_error / snap_bound * new_dyn_max_v);
                    }
                    else
                        return (float)(roll_c.snapping_Kp * angle_error / snap_bound * max_roll_v * 0.05);
                }
                else
                {
                    cubic = true;
                    double t = Math.Pow(Math.Abs(angle_error / cubic_k), 1.0 / 3.0);
                    if (t < roll_relax_frame * TimeWarp.fixedDeltaTime)
                    {
                        // relaxation mode
                        return (float)(roll_relax_Kp * angle_error / TimeWarp.fixedDeltaTime);
                    }
                    else
                    {
                        double t_next = Math.Max(0.0, t - TimeWarp.fixedDeltaTime);
                        return (float)(Math.Sign(angle_error) * cubic_k * (Math.Pow(t, 3) - Math.Pow(t_next, 3)) / TimeWarp.fixedDeltaTime);
                    }
                }
            }
        }

        #endregion

        #region AoAs

        Matrix aoa_A = new Matrix(2, 2);
        Matrix aoa_B = new Matrix(2, 1);
        Matrix eq_x;

        public float get_desired_aoa(LinearSystemModel rotation_model, double desired_v, double desired_aoa_deriv)
        {
            aoa_A[0, 0] = rotation_model.A[0, 0];
            aoa_A[0, 1] = rotation_model.A[0, 2] + rotation_model.A[0, 3] + rotation_model.B[0, 0];
            aoa_A[1, 0] = rotation_model.A[1, 0];
            aoa_A[1, 1] = rotation_model.A[1, 2] + rotation_model.A[1, 3] + rotation_model.B[1, 0];
            aoa_B[0, 0] = desired_aoa_deriv - desired_v - rotation_model.C[0, 0];
            aoa_B[1, 0] = -rotation_model.C[1, 0];
            aoa_A.old_lu = true;
            try
            {
                eq_x = aoa_A.SolveWith(aoa_B);
                double desired_aoa = eq_x[0, 0];
                return (float)desired_aoa;
            }
            catch (MSingularException) { return float.NaN; }
        }

        [AutoGuiAttr("desired_aoa", false, "G5")]
        public float desired_aoa;

        [AutoGuiAttr("desired_sideslip", false, "G5")]
        public float desired_sideslip;

        public float max_lift_acceleration(int axis)
        {
            if (axis == PITCH)
            {
                return (float)(pitch_c.res_equilibr_v_upper * imodel.surface_v_magnitude - imodel.prev_pitch_gravity_acc - imodel.prev_pitch_noninert_acc);
            }
            else
                if (axis == YAW)
            {
                return (float)(yaw_c.res_equilibr_v_upper * imodel.surface_v_magnitude - imodel.prev_yaw_gravity_acc - imodel.prev_yaw_noninert_acc);
            }
            return 0.0f;
        }

        #endregion

    }
}
