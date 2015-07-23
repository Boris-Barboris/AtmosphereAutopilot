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
    public sealed class SideslipController : SISOController
    {
        InstantControlModel imodel;
        YawAngularVelocityController v_controller;

        PController pid = new PController();

        internal SideslipController(Vessel v) :
            base(v, "Sideslip controller", 88437222) { AutoTrim = true; }

        public override void InitializeDependencies(Dictionary<Type, AutopilotModule> modules)
        {
            imodel = modules[typeof(InstantControlModel)] as InstantControlModel;
            v_controller = modules[typeof(YawAngularVelocityController)] as YawAngularVelocityController;
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

        double time_in_regime = 0.0;

        [AutoGuiAttr("sideslip angle", false, "G8")]
        float sideslip;

        [AutoGuiAttr("output vel", false, "G8")]
        float output;

        /// <summary>
        /// Main control function
        /// </summary>
        /// <param name="cntrl">Control state to change</param>
        public override float ApplyControl(FlightCtrlState cntrl, float target_value)
        {
            const float degree_to_rad = (float)Math.PI / 180.0f;

            sideslip = -imodel.AoA(YAW);

            double user_input = ControlUtils.get_neutralized_user_input(cntrl, YAW);
            if (user_input != 0.0)
                desired_sideslip = (float)(user_input * fbw_max_sideslip * degree_to_rad);
            else
                desired_sideslip = target_value;

            desired_v = (float)pid.Control(sideslip, desired_sideslip);

            float error = desired_sideslip - sideslip;

            //ControlUtils.neutralize_user_input(cntrl, YAW);
            v_controller.ApplyControl(cntrl, output);

            // check if we're stable on given input value
            if (AutoTrim)
            {
                if (Math.Abs(sideslip) < 5e-3)
                {
                    time_in_regime += TimeWarp.fixedDeltaTime;
                }
                else
                {
                    time_in_regime = 0.0;
                }

                if (time_in_regime >= 5.0)
                    ControlUtils.set_trim(YAW, imodel.ControlInputHistory(YAW).Average());
            }

            return output;
        }


        #region Parameters

        [VesselSerializable("kp_vel_factor")]
        [GlobalSerializable("kp_vel_factor")]
        [AutoGuiAttr("KP velocity factor", true, "G6")]
        float kp_vel_factor = 0.5f;

        [GlobalSerializable("fbw_max_sideslip")]
        [VesselSerializable("fbw_max_sideslip")]
        [AutoGuiAttr("max Sideslip in degrees", true, "G6")]
        float fbw_max_sideslip = 10.0f;

        [AutoGuiAttr("DEBUG desired_v", false, "G8")]
        float desired_v;

        [AutoGuiAttr("DEBUG desired_sideslip", false, "G8")]
        float desired_sideslip;

        [GlobalSerializable("AutoTrim")]
        [AutoGuiAttr("AutoTrim", true, null)]
        public bool AutoTrim { get; set; }

        #endregion
    }
}
