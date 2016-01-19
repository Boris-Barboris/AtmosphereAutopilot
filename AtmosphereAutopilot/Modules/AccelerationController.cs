using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;

namespace AtmosphereAutopilot
{

    public sealed class AccelerationController: AutopilotModule
    {
        internal AccelerationController(Vessel v)
            : base(v, 88437225, "Acceleration controller")
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

        protected override void OnActivate()
        {
            imodel.Activate();
            aoa_c.Activate();
            yaw_c.Activate();
            roll_c.Activate();
        }

        protected override void OnDeactivate()
        {
            imodel.Deactivate();
            aoa_c.Deactivate();
            yaw_c.Deactivate();
            roll_c.Deactivate();
        }

        /// <summary>
        /// Main control function
        /// </summary>
        /// <param name="desired_acceleration">Desired acceleration in planet (rotating) reference frame.</param>
        /// <param name="jerk">Desired acceleration derivative.</param>
        public void ApplyControl(FlightCtrlState state, Vector3d desired_acceleration, Vector3d jerk)
        {
            target_acc = desired_acceleration;
            current_acc = imodel.sum_acc;

            // we need aoa moderation for AoA controllers to work
            pitch_c.moderate_aoa = true;
            pitch_c.moderate_g = true;
            yaw_c.moderate_aoa = true;
            yaw_c.moderate_g = true;

            Vector3d target_lift_acc = target_acc - imodel.gravity_acc - imodel.noninert_acc;
            Vector3d target_normal_lift_acc = target_lift_acc - Vector3d.Project(target_lift_acc, imodel.surface_v);
            Vector3d desired_right_direction = Vector3d.Cross(target_normal_lift_acc, vessel.ReferenceTransform.up).normalized;

            double craft_max_g = Math.Max(0.01, pitch_c.max_aoa_v * imodel.surface_v.magnitude -
                imodel.prev_pitch_gravity_acc - imodel.prev_pitch_noninert_acc);

            // let's apply roll to maintain desired_right_direction
            double new_roll_angle = Math.Sign(Vector3d.Dot(vessel.ReferenceTransform.right, target_normal_lift_acc)) *
                Math.Acos(Math.Min(Math.Max(Vector3d.Dot(desired_right_direction, vessel.ReferenceTransform.right), -1.0), 1.0));
            // rolling to pitch up is not always as efficient as pitching down
            double spine_up = Vector3d.Dot(desired_right_direction, Vector3d.Cross(imodel.surface_v, imodel.gravity_acc));
            if (target_normal_lift_acc.magnitude < craft_max_g * 0.5 || !allow_roll_overs)
                if (Math.Abs(new_roll_angle) > 90.0 * dgr2rad && spine_up < 0.0)
                    new_roll_angle = new_roll_angle - 180.0 * dgr2rad * Math.Sign(new_roll_angle);
            // filter it
            if ((Math.Abs(new_roll_angle) < roll_error_filter_margin) && (Math.Abs(roll_angle) < roll_error_filter_margin))
               roll_angle = Common.simple_filter(new_roll_angle, roll_angle, roll_error_filter_k);
            else
                roll_angle = new_roll_angle;
            // generate desired roll angular_v
            roll_c.user_controlled = false;
            roll_c.ApplyControl(state, get_desired_roll_v(roll_angle));

            // now let's apply pitch and yaw AoA controls

            // pitch AoA

            Vector3d current_lift_dir = imodel.pitch_tangent;
            Vector3d current_normal_lift = current_lift_dir - Vector3d.Project(current_lift_dir, imodel.surface_v);
            desired_pitch_lift = 0.0;
            if (Math.Abs(roll_angle) < 90.0 * dgr2rad)
                desired_pitch_lift = Vector3.Dot(imodel.pitch_tangent, target_normal_lift_acc);
            else
                desired_pitch_lift = 0.0;
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
            //}
            //desired_sideslip = 0.0f;
            side_c.user_controlled = false;
            side_c.ApplyControl(state, desired_sideslip, 0.0f);
        }

        //[AutoGuiAttr("target_acc", false, "G3")]
        protected Vector3d target_acc;

        //[AutoGuiAttr("current_acc", false, "G3")]
        protected Vector3d current_acc;

        //[AutoGuiAttr("desired_yaw_lift", false, "G5")]
        protected double desired_yaw_lift;

        //[AutoGuiAttr("desired_yaw_acc", false, "G5")]
        protected double desired_yaw_acc;

        //[AutoGuiAttr("desired_yaw_v", false, "G5")]
        protected double desired_yaw_v;

        //[AutoGuiAttr("sideslip_filter_k", true, "G4")]
        protected double sideslip_filter_k = 10.0;

        [AutoGuiAttr("desired_pitch_lift", false, "G5")]
        protected double desired_pitch_lift;

        [AutoGuiAttr("desired_pitch_acc", false, "G5")]
        protected double desired_pitch_acc;

        [AutoGuiAttr("desired_pitch_v", false, "G5")]
        protected double desired_pitch_v;

        public bool allow_roll_overs = true;

        # region Roll

        Matrix roll_state_m = new Matrix(3, 1);
        Matrix roll_input_m = new Matrix(3, 1);

        [AutoGuiAttr("roll_acc_factor", false, "G5")]
        public double roll_acc_factor = 0.0;

        [AutoGuiAttr("roll_acc_filter", true, "G5")]
        protected double roll_acc_filter = 4.0;

        [AutoGuiAttr("roll_cubic_K", true, "G5")]
        protected double roll_cubic_K = 0.3;

        [AutoGuiAttr("roll_cubic_relax_frame", true, "G5")]
        protected double roll_relax_frame = 10.0;

        [AutoGuiAttr("roll_relax_Kp", true, "G5")]
        protected double roll_relax_Kp = 0.1;

        [AutoGuiAttr("roll_error_filter_margin", true, "G5")]
        protected double roll_error_filter_margin = 3.0 * dgr2rad;

        [AutoGuiAttr("roll_error_filter_k", true, "G5")]
        protected double roll_error_filter_k = 0.5;

        [AutoGuiAttr("max_roll_v", false, "G5")]
        public double max_roll_v;

        [AutoGuiAttr("roll_angle", false, "G5")]
        protected double roll_angle = 0.0;

        [AutoGuiAttr("cubic", false)]
        protected bool cubic;

        [AutoGuiAttr("snapping_boundary", true, "G5")]
        protected double snapping_boundary = 3.0 * dgr2rad;

        float get_desired_roll_v(double angle_error)
        {
            roll_state_m[0, 0] = 0.0;
            roll_state_m[1, 0] = 1.0;
            roll_state_m[2, 0] = 1.0;
            roll_input_m[0, 0] = 1.0;
            double roll_max_acc = imodel.roll_rot_model_gen.eval_row(0, roll_state_m, roll_input_m);
            roll_acc_factor = Common.simple_filter(Math.Abs(roll_max_acc), roll_acc_factor, roll_acc_filter);

            // calculate anti-overshooting perameters
            if (angle_error >= 0.0)
                max_roll_v = Math.Min(roll_c.max_input_v, roll_c.max_v_construction);
            else
                max_roll_v = Math.Abs(Math.Max(roll_c.min_input_v, -roll_c.max_v_construction));
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
                if (Math.Abs(angle_error) < snapping_boundary)
                {
                    // error is too small, let's use snapping code
                    double new_dyn_max_v = Math.Sqrt(snapping_boundary * roll_acc_factor);
                    if (!double.IsNaN(new_dyn_max_v))
                    {
                        new_dyn_max_v = Common.Clamp(new_dyn_max_v, max_roll_v);
                        return (float)(roll_c.snapping_Kp * angle_error / snapping_boundary * new_dyn_max_v);
                    }
                    else
                        return (float)(roll_c.snapping_Kp * angle_error / snapping_boundary * max_roll_v * 0.05);
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
        protected float desired_aoa;

        [AutoGuiAttr("desired_sideslip", false, "G5")]
        protected float desired_sideslip;

        public float max_lift_acceleration(int axis)
        {
            if (axis == PITCH)
            {
                return (float)(pitch_c.max_aoa_v * imodel.surface_v_magnitude - imodel.prev_pitch_gravity_acc - imodel.prev_pitch_noninert_acc);
            }
            else
                if (axis == YAW)
                {
                    return (float)(yaw_c.max_aoa_v * imodel.surface_v_magnitude - imodel.prev_yaw_gravity_acc - imodel.prev_yaw_noninert_acc);
                }
            return 0.0f;
        }

        #endregion

    }
}
