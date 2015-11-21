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
    public sealed class StandardFlyByWire : StateController
    {
        PitchAngularVelocityController pc;
        RollAngularVelocityController rc;
		YawAngularVelocityController yvc;
        SideslipController yc;
        ProgradeThrustController tc;
        AutopilotModule[] gui_list = new AutopilotModule[5];

        internal StandardFlyByWire(Vessel v) :
            base(v, "Standard Fly-By-Wire", 44421322) { }

        public override void InitializeDependencies(Dictionary<Type, AutopilotModule> modules)
        {
            gui_list[0] = pc = modules[typeof(PitchAngularVelocityController)] as PitchAngularVelocityController;
            gui_list[1] = rc = modules[typeof(RollAngularVelocityController)] as RollAngularVelocityController;
			gui_list[2] = yvc = modules[typeof(YawAngularVelocityController)] as YawAngularVelocityController;
            gui_list[3] = yc = modules[typeof(SideslipController)] as SideslipController;
            gui_list[4] = tc = modules[typeof(ProgradeThrustController)] as ProgradeThrustController;
        }

        protected override void OnActivate() 
        {
            pc.Activate();
            pc.user_controlled = true;
            rc.Activate();
            rc.user_controlled = true;
			yvc.Activate();
			yvc.user_controlled = rocket_mode;
            yc.Activate();
            yc.user_controlled = true;
            tc.Activate();
            MessageManager.post_status_message("Standard Fly-By-Wire enabled");
        }

        protected override void OnDeactivate()
        {
            pc.Deactivate();
            rc.Deactivate();
            yvc.Deactivate();
            yc.Deactivate();
            MessageManager.post_status_message("Standard Fly-By-Wire disabled");
        }

		[VesselSerializable("rocket_mode")]
		[AutoGuiAttr("Rocket mode", true)]
		public bool rocket_mode = false;

        [AutoGuiAttr("Moderation", true)]
        public bool moderation_switch
        {
            get
            {
                return (pc.moderate_aoa || pc.moderate_g || yvc.moderate_aoa || yvc.moderate_g);
            }
            set
            {
                if (value != moderation_switch)
                {
                    MessageManager.post_status_message(value ? "Moderation enabled" : "Moderation disabled");
                    pc.moderate_aoa = pc.moderate_g = yvc.moderate_aoa = yvc.moderate_g = value;
                }
            }
        }

        [GlobalSerializable("moderation_keycode")]
        protected KeyCode moderation_keycode = KeyCode.O;

        [AutoGuiAttr("Cruise control", true)]
        public bool cruise_control = false;

        [VesselSerializable("cruise_speed")]
        [AutoGuiAttr("Cruise speed", true, "G4")]
        public float cruise_speed = 100.0f;

        public override void OnUpdate()
        {
            if (Input.GetKeyDown(moderation_keycode))
                moderation_switch = !moderation_switch;
        }

        /// <summary>
        /// Main control function
        /// </summary>
        /// <param name="cntrl">Control state to change</param>
        public override void ApplyControl(FlightCtrlState cntrl)
        {
            if (cruise_control)
                tc.ApplyControl(cntrl, cruise_speed);

            if (vessel.LandedOrSplashed)
                return;

            pc.ApplyControl(cntrl, 0.0f);
			if (rocket_mode)
			{
				yvc.user_controlled = true;
				yvc.ApplyControl(cntrl, 0.0f);
			}
			else
				yc.ApplyControl(cntrl, 0.0f);
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
