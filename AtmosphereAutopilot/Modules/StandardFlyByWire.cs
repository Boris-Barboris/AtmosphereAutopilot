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
    public sealed class StandardFlyByWire : StateController
    {
        PitchAngularVelocityController pc;
        RollAngularVelocityController rc;
        YawAngularVelocityController yvc;
        SideslipController yc;
        ProgradeThrustController tc;
        FlightModel im;
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
            im = modules[typeof(FlightModel)] as FlightModel;
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
            pc.neutral_offset = 0.0f;
            pc.Deactivate();
            rc.Deactivate();
            yvc.Deactivate();
            yc.Deactivate();
            MessageManager.post_status_message("Standard Fly-By-Wire disabled");
        }

        [VesselSerializable("rocket_mode")]
        [AutoGuiAttr("Rocket mode", true)]
        public bool rocket_mode = false;

        public bool RocketMode
        {
            get
            {
                return rocket_mode;
            }
            set
            {
                if (value != rocket_mode)
                {
                    MessageManager.post_status_message(value ? "Rocket mode enabled" : "Rocket mode disabled");
                    rocket_mode = value;
                }
            }
        }

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
        [AutoHotkeyAttr("FBW moderation")]
        static KeyCode moderation_keycode = KeyCode.O;

        [GlobalSerializable("rocket_mode_keycode")]
        [AutoHotkeyAttr("FBW rocket mode")]
        static KeyCode rocket_mode_keycode = KeyCode.None;

        [GlobalSerializable("coord_turn_keycode")]
        [AutoHotkeyAttr("FBW coord turn")]
        static KeyCode coord_turn_keycode = KeyCode.None;

        [AutoGuiAttr("Coordinated turn", true)]
        [VesselSerializable("coord_turn")]
        public bool coord_turn = false;

        public bool Coord_turn
        {
            get
            {
                return coord_turn;
            }
            set
            {
                if (value != coord_turn)
                {
                    MessageManager.post_status_message(value ? "Coord turn enabled" : "Coord turn disabled");
                    coord_turn = value;
                }
            }
        }

        public override void OnUpdate()
        {
            bool changed = false;
            if (Input.GetKeyDown(moderation_keycode))
            {
                moderation_switch = !moderation_switch;
                changed = true;
            }
            if (Input.GetKeyDown(rocket_mode_keycode))
            {
                RocketMode = !rocket_mode;
                changed = true;
            }
            if (Input.GetKeyDown(coord_turn_keycode))
            {
                Coord_turn = !coord_turn;
                changed = true;
            }
            if (changed)
                AtmosphereAutopilot.Instance.mainMenuGUIUpdate();
        }

        bool landed = false;
        bool need_restore = false;
        float time_after_takeoff = 0.0f;
        bool aoa_moder = true;
        bool g_moder = true;

        /// <summary>
        /// Main control function
        /// </summary>
        /// <param name="cntrl">Control state to change</param>
        public override void ApplyControl(FlightCtrlState cntrl)
        {
            if (vessel.LandedOrSplashed)
            {
                landed = true;
                time_after_takeoff = 0.0f;
                return;
            }

            // disable pitch moderation for two seconds after take-off
            if (landed || need_restore)
            {
                if (landed && !need_restore)
                {
                    aoa_moder = pc.moderate_aoa;
                    g_moder = pc.moderate_g;
                    pc.moderate_aoa = false;
                    pc.moderate_g = false;
                    landed = false;
                    need_restore = true;
                }
                if (time_after_takeoff > 1.5f)
                {
                    pc.moderate_aoa = aoa_moder;
                    pc.moderate_g = g_moder;
                    need_restore = false;
                }
                else
                    time_after_takeoff += TimeWarp.fixedDeltaTime;
            }

            if (tc.spd_control_enabled)
                tc.ApplyControl(cntrl, tc.setpoint.mps());

			pc.user_controlled = true;
            if (coord_turn)
            {
                // account for yaw velocity in pitch neutral offset to assist coordinated turn
                Vector3 up_level_dir = Vector3.ProjectOnPlane(vessel.ReferenceTransform.position - vessel.mainBody.position,
                    vessel.ReferenceTransform.up).normalized;
                float yaw_v_vert_project = Vector3.Dot(im.AngularVel(YAW) * vessel.ReferenceTransform.right, up_level_dir);
                float pitch_vert_project = Vector3.Dot(up_level_dir, -vessel.ReferenceTransform.forward);
                if (pitch_vert_project > 0.0f)
                {
                    float level_pitch_vel = -yaw_v_vert_project / pitch_vert_project;
                    pc.neutral_offset = level_pitch_vel;
                }
                else
                    pc.neutral_offset = 0.0f;
            }
            else
                pc.neutral_offset = 0.0f;
            pc.ApplyControl(cntrl, 0.0f);

			if (rocket_mode)
			{
				yvc.user_controlled = true;
				yvc.ApplyControl(cntrl, 0.0f);
			}
			else
			{
				yc.user_controlled = true;
				yc.ApplyControl(cntrl, 0.0f, 0.0f);
			}
			rc.user_controlled = true;
            rc.ApplyControl(cntrl, 0.0f);
        }

        protected override void _drawGUI(int id)
        {
            close_button();
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
            tc.SpeedCtrlGUIBlock();
            GUILayout.EndVertical();
            GUI.DragWindow();
        }
    }
}
