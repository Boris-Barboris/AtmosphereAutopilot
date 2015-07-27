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
                lin_model = imodel.pitch_rot_model;
            }
            else
                if (axis == YAW)
                {
                    v_controller = modules[typeof(YawAngularVelocityController)] as PitchYawAngularVelocityController;
                    lin_model = imodel.yaw_rot_model;
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

        protected LinearSystemModel lin_model;

        /// <summary>
        /// Main control function
        /// </summary>
        /// <param name="cntrl">Control state to change</param>
        public override float ApplyControl(FlightCtrlState cntrl, float target_value)
        {
            cur_aoa = imodel.AoA(axis);

            float user_input = Common.Clampf(ControlUtils.get_neutralized_user_input(cntrl, YAW), 1.0f);
            if (user_controlled || user_input != 0.0f)
                desired_aoa = user_input * v_controller.res_max_aoa;
            else
                if (user_input < 0.0)
                    desired_aoa = -user_input * v_controller.res_min_aoa;
                else
                    desired_aoa = (float)Common.Clamp(target_value, v_controller.res_min_aoa, v_controller.res_max_aoa);

            // Let's find balance angular v on desired_aoa
            float des_aoa_equilibr_v = -(float)(lin_model.C[0, 0] + lin_model.A[0, 0] * desired_aoa);
            float transit_v = v_controller.transit_max_v;
            float error = desired_aoa - cur_aoa;
            float relax_k = error * 2.0f / (v_controller.res_max_aoa - v_controller.res_min_aoa);
            if (float.IsInfinity(relax_k) || float.IsNaN(relax_k))
                relax_k = 0.0f;
            relax_k = Common.Clampf(relax_k, 1.0f);

            output_v = relax_k * transit_v + (1.0f - Math.Abs(relax_k)) * des_aoa_equilibr_v;

            ControlUtils.neutralize_user_input(cntrl, axis);
            v_controller.user_controlled = false;
            v_controller.ApplyControl(cntrl, output_v);

            return output_v;
        }
    }

    public sealed class SideslipController : AoAController
    {
        internal SideslipController(Vessel v) :
            base(v, "Sideslip controller", 88437222, YAW) { }
    }
}
