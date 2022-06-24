/*
Atmosphere Autopilot, plugin for Kerbal Space Program.
Copyright (C) 2015-2022, Baranin Alexander aka Boris-Barboris.

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
using UnityEngine;

namespace AtmosphereAutopilot
{
    /// <summary>
    /// Holds specific AoA
    /// </summary>
    public sealed class AoAHoldController : StateController
    {
        internal AoAHoldController(Vessel v)
            : base(v, "AoA-hold controller", 80932038)
        { }

        FlightModel imodel;
        PitchAoAController aoa_c;
        SideslipController side_c;
        ProgradeThrustController thrust_c;
        PitchAngularVelocityController pitch_c;
        RollAngularVelocityController roll_c;
        YawAngularVelocityController yaw_c;

        public override void InitializeDependencies(Dictionary<Type, AutopilotModule> modules)
        {
            imodel = modules[typeof(FlightModel)] as FlightModel;
            aoa_c = modules[typeof(PitchAoAController)] as PitchAoAController;
            side_c = modules[typeof(SideslipController)] as SideslipController;
            pitch_c = modules[typeof(PitchAngularVelocityController)] as PitchAngularVelocityController;
            roll_c = modules[typeof(RollAngularVelocityController)] as RollAngularVelocityController;
            yaw_c = modules[typeof(YawAngularVelocityController)] as YawAngularVelocityController;
            thrust_c = modules[typeof(ProgradeThrustController)] as ProgradeThrustController;
        }

        bool aoa_moderation_saved = false;

        protected override void OnActivate()
        {
            imodel.Activate();
            aoa_c.Activate();
            yaw_c.Activate();
            roll_c.Activate();
            // save moderation states
            aoa_moderation_saved = pitch_c.moderate_aoa;
            thrust_c.Activate();
            MessageManager.post_status_message("AoA-hold enabled");

            // initialize desired AoA from the current one
            desired_aoa.Value = imodel.AoA(PITCH) * rad2dgr;
            // if current AoA is large, assume that the user does not want
            // moderation
            if (aoa_moderation_saved && Mathf.Abs(desired_aoa) > pitch_c.max_aoa)
                moderation_switch = false;
        }

        protected override void OnDeactivate()
        {
            imodel.Deactivate();
            aoa_c.Deactivate();
            yaw_c.Deactivate();
            roll_c.Deactivate();
            // restore moderation states
            pitch_c.moderate_aoa = aoa_moderation_saved;
            thrust_c.Deactivate();
            MessageManager.post_status_message("AoA-hold disabled");
        }

        // degrees
        public DelayedFieldFloat desired_aoa = new DelayedFieldFloat(0.0f, "G4");

        [GlobalSerializable("use_keys")]
        [AutoGuiAttr("use keys", true)]
        public static bool use_keys = true;

        [AutoGuiAttr("Pitch moderation", true)]
        public bool moderation_switch
        {
            get
            {
                return (pitch_c.moderate_aoa || pitch_c.moderate_g);
            }
            set
            {
                if (value != moderation_switch)
                {
                    MessageManager.post_status_message(value ? "Pitch moderation enabled" : "Pitch moderation disabled");
                    pitch_c.moderate_aoa = pitch_c.moderate_g = value;
                }
            }
        }

        [AutoGuiAttr("hotkey sensitivity", true, "G4")]
        [GlobalSerializable("hotkey_desired_aoa_sens")]
        public static float hotkey_desired_aoa_sens = 3.0f; // degrees/sec

        public override void ApplyControl(FlightCtrlState cntrl)
        {
            if (vessel.LandedOrSplashed())
                return;

            if (thrust_c.spd_control_enabled)
                thrust_c.ApplyControl(cntrl, thrust_c.setpoint.mps());

            if (use_keys)
                ControlUtils.neutralize_user_input(cntrl, PITCH);

            aoa_c.user_controlled = false;
            if (pitch_c.moderate_aoa)
            {
                // limit desired AoA with craft's "input" AoA limit.
                desired_aoa.Value = Common.Clampf(desired_aoa, pitch_c.max_aoa);
            }
            aoa_c.ApplyControl(cntrl, desired_aoa * dgr2rad, 0.0f);
            side_c.user_controlled = true;
            side_c.ApplyControl(cntrl, 0.0f, 0.0f);
            roll_c.user_controlled = true;
            roll_c.ApplyControl(cntrl, 0.0f);
        }

        public override void OnUpdate()
        {
            if (use_keys && !FlightDriver.Pause &&
                InputLockManager.IsUnlocked(ControlTypes.PITCH))
            {
                bool pitch_key_pressed = false;
                float pitch_change_sign = 0.0f;
                // Pitch
                if (GameSettings.PITCH_UP.GetKey() && !GameSettings.MODIFIER_KEY.GetKey())
                {
                    pitch_change_sign = 1.0f;
                    pitch_key_pressed = true;
                }
                else if (GameSettings.PITCH_DOWN.GetKey() && !GameSettings.MODIFIER_KEY.GetKey())
                {
                    pitch_change_sign = -1.0f;
                    pitch_key_pressed = true;
                }

                if (pitch_key_pressed)
                {
                    desired_aoa.Value = desired_aoa + pitch_change_sign * Time.deltaTime *
                        hotkey_desired_aoa_sens;
                }
            }
            bool changed = false;
            if (Input.GetKeyDown(StandardFlyByWire.moderation_keycode))
            {
                moderation_switch = !moderation_switch;
                changed = true;
            }
            if (changed)
                AtmosphereAutopilot.Instance.mainMenuGUIUpdate();
        }

        protected override void OnGUICustomAlways()
        {
            desired_aoa.OnUpdate();
        }

        protected override void _drawGUI(int id)
        {
            close_button();
            GUILayout.BeginVertical();
            AutoGUI.AutoDrawObject(this);
            desired_aoa.DisplayLayout(GUIStyles.textBoxStyle);
            thrust_c.SpeedCtrlGUIBlock();
            GUILayout.EndVertical();
            GUI.DragWindow();
        }
    }
}
