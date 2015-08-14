/*
Atmosphere Autopilot, plugin for Kerbal Space Program.
Copyright (C) 2015, Baranin Alexander aka Boris-Barboris.
 
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

    public abstract class AoAController : SISOController
    {
        InstantControlModel imodel;
        PitchYawAngularVelocityController v_controller;

        int axis;

        internal AoAController(Vessel v, string cntrl_name, int wnd_id, int axis) : base(v, cntrl_name, wnd_id)
        {
            this.axis = axis;
        }

        public override void InitializeDependencies(Dictionary<Type, AutopilotModule> modules)
        {
            imodel = modules[typeof(InstantControlModel)] as InstantControlModel;
            if (axis == PITCH)
            {
                v_controller = modules[typeof(PitchAngularVelocityController)] as PitchYawAngularVelocityController;
                lin_model = imodel.pitch_rot_model_gen;
            }
            else
                if (axis == YAW)
                {
                    v_controller = modules[typeof(YawAngularVelocityController)] as PitchYawAngularVelocityController;
                    lin_model = imodel.yaw_rot_model_gen;
                }
        }

        protected override void OnActivate()
        {
            imodel.Activate();
            v_controller.Activate();
        }

        protected override void OnDeactivate()
        {
            imodel.Deactivate();
            v_controller.Deactivate();
        }

        [AutoGuiAttr("AoA", false, "G6")]
        protected float cur_aoa;

        [AutoGuiAttr("desired_aoa", false, "G6")]
        protected float desired_aoa;

        [AutoGuiAttr("output_v", false, "G6")]
        protected float output_v;

        [AutoGuiAttr("des_aoa_equilibr_v", false, "G6")]
        protected float des_aoa_equilibr_v;

        [AutoGuiAttr("filter_k", true, "G6")]
        protected float filter_k = 4.0f;

        protected LinearSystemModel lin_model;

        Matrix eq_A = new Matrix(2, 2);
        Matrix eq_b = new Matrix(2, 1);
        Matrix eq_x;

        /// <summary>
        /// Main control function
        /// </summary>
        /// <param name="cntrl">Control state to change</param>
        public override float ApplyControl(FlightCtrlState cntrl, float target_value)
        {
            if (imodel.dyn_pressure <= 100.0 || !v_controller.moderate_aoa)
            {
                v_controller.user_controlled = true;
                v_controller.ApplyControl(cntrl, 0.0f);
                return 0.0f;
            }

            cur_aoa = imodel.AoA(axis);

            float user_input = Common.Clampf(ControlUtils.get_neutralized_user_input(cntrl, YAW), 1.0f);
            if (user_controlled || user_input != 0.0f)
            {
                if (user_input >= 0.0f)
                    desired_aoa = user_input * v_controller.res_max_aoa;
                else
                    desired_aoa = -user_input * v_controller.res_min_aoa;
                user_controlled = true;
            }
            else
                desired_aoa = (float)Common.Clamp(target_value, v_controller.res_min_aoa, v_controller.res_max_aoa);

            // Let's find balance angular v on desired_aoa
            if (Math.Abs(cur_aoa) < 0.3f)
            {
                eq_A[0, 0] = lin_model.A[0, 1];
                eq_A[0, 1] = lin_model.A[0, 2];
                eq_A[1, 0] = lin_model.A[1, 1];
                eq_A[1, 1] = lin_model.A[1, 2] + lin_model.B[1, 0];
                eq_b[0, 0] = -(lin_model.A[0, 0] * desired_aoa + lin_model.C[0, 0]);
                eq_b[1, 0] = -(lin_model.A[1, 0] * desired_aoa + lin_model.C[1, 0]);
                eq_A.old_lu = true;
                try
                {
                    eq_x = eq_A.SolveWith(eq_b);
                    double new_eq_v = eq_x[0, 0];
                    if (!double.IsInfinity(new_eq_v) && !double.IsNaN(new_eq_v))
                        des_aoa_equilibr_v = (float)Common.simple_filter(new_eq_v, des_aoa_equilibr_v, filter_k);
                }
                catch (MSingularException) { }
            }
            else
                des_aoa_equilibr_v = 0.0f;

            //des_aoa_equilibr_v += (float)get_roll_aoa_deriv(desired_aoa);

            float transit_v = v_controller.transit_max_v;
            float error = desired_aoa - cur_aoa;
            float relax_k = 0.0f;
            if (v_controller.res_max_aoa - v_controller.res_min_aoa > 0.0f)
                relax_k = error * 2.0f / (v_controller.res_max_aoa - v_controller.res_min_aoa);
            else
                if (v_controller.max_aoa > 0.0f)
                    relax_k = error / v_controller.max_aoa;
                else
                    relax_k = error / 0.2f;
            relax_k = Common.Clampf(relax_k, 1.0f);

            output_v = relax_k * transit_v + des_aoa_equilibr_v;

            ControlUtils.neutralize_user_input(cntrl, axis);
            v_controller.user_controlled = false;
            v_controller.ApplyControl(cntrl, output_v);

            return output_v;
        }

        double get_roll_aoa_deriv(float desired_aoa)
        {
            Vector3 pitch_aoa = new Vector3(axis == PITCH ? desired_aoa : imodel.AoA(PITCH), 0.0f, 0.0f);
            Vector3 yaw_aoa = new Vector3(0.0f, axis == YAW ? desired_aoa : imodel.AoA(YAW), 0.0f);
            Vector3 ang_v = new Vector3(0.0f, 0.0f, imodel.AngularVel(ROLL));
            Vector3 plane_vel = Vector3.Cross(ang_v, pitch_aoa + yaw_aoa);
            if (axis == PITCH)
                return Vector3.Dot(Vector3.right, plane_vel);
            if (axis == YAW)
                return Vector3.Dot(Vector3.up, plane_vel);
            return 0.0;
        }
    }

    public sealed class SideslipController : AoAController
    {
        internal SideslipController(Vessel v) :
            base(v, "Sideslip controller", 88437222, YAW) { }
    }

	public sealed class PitchAoAController : AoAController
	{
		internal PitchAoAController(Vessel v) :
			base(v, "AoA controller", 88437223, PITCH) { }
	}
}
