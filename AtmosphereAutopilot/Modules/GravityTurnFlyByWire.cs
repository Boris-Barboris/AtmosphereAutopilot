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
    public sealed class GravityTurnFlyByWire : StateController
    {
        PitchAngularVelocityController pc;
        RollAngularVelocityController rc;
        YawAngularVelocityController yvc;
        PitchAoAController ac;
        SideslipController yc;
        AutopilotModule[] gui_list = new AutopilotModule[5];

        internal GravityTurnFlyByWire(Vessel v) :
            base(v, "Gravity turn Fly-By-Wire", 44421340) { }

        public override void InitializeDependencies(Dictionary<Type, AutopilotModule> modules)
        {
            gui_list[0] = pc = modules[typeof(PitchAngularVelocityController)] as PitchAngularVelocityController;
            gui_list[1] = rc = modules[typeof(RollAngularVelocityController)] as RollAngularVelocityController;
            gui_list[2] = yvc = modules[typeof(YawAngularVelocityController)] as YawAngularVelocityController;
            gui_list[3] = ac = modules[typeof(PitchAoAController)] as PitchAoAController;
            gui_list[4] = yc = modules[typeof(SideslipController)] as SideslipController;
        }

        protected override void OnActivate() 
        {
            yc.Activate();
            yc.user_controlled = true;
            ac.Activate();
            ac.user_controlled = true;
            rc.Activate();
            rc.user_controlled = true;
            MessageManager.post_status_message("Gravity turn Fly-By-Wire enabled");
        }

        protected override void OnDeactivate()
        {
            rc.Deactivate();
            yc.Deactivate();
            ac.Deactivate();
            MessageManager.post_status_message("Gravity turn Fly-By-Wire disabled");
        }

        /// <summary>
        /// Main control function
        /// </summary>
        /// <param name="cntrl">Control state to change</param>
        public override void ApplyControl(FlightCtrlState cntrl)
        {
            if (vessel.LandedOrSplashed)
                return;

            ac.ApplyControl(cntrl, 0.0f, 0.0f);
            yc.ApplyControl(cntrl, 0.0f, 0.0f);
            rc.ApplyControl(cntrl, 0.0f);
        }

        protected override void _drawGUI(int id)
        {
            GUILayout.BeginVertical();
            foreach (var module in gui_list)
            {
                bool is_shown = GUILayout.Toggle(module.IsShown(), module.ModuleName + " GUI", GUIStyles.toggleButtonStyle);
                if (is_shown)
                    module.ShowGUI();
                else
                    module.UnShowGUI();
            }
            GUILayout.Space(5.0f);
            AutoGUI.AutoDrawObject(this);
            GUILayout.EndVertical();
            GUI.DragWindow();
        }
    }
}
