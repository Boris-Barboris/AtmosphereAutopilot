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
    class AppLauncherWindow : GUIWindow
    {
        public AppLauncherWindow() :
            base("", 3920049, new Rect(Screen.width - 280, 38, 230, 30))
        { }

        public void set_left(int left)
        {
            window.x = left;
        }

        public bool show_while_hover = false;

        protected override void _drawGUI(int id)
        {
            GUILayout.BeginVertical();

            AtmosphereAutopilot aa = AtmosphereAutopilot.Instance;
            foreach (var pair in aa.getVesselModules(aa.ActiveVessel))
            {
                draw_row(pair.Value, pair.Key.Name);
            }
            // custom behaviour for hotkey manager
            AutoHotkey hm = aa.hotkeyManager;
            if (hm != null)
            {
                draw_row(hm, "Hotkeys manager");
            }

            GUILayout.EndVertical();
        }

        void draw_row(IWindow gui, string type_name)
        {
            bool shown = gui.IsShown();
            string label = (gui as GUIWindow) != null ?
                (gui as GUIWindow).WindowName :
                type_name;
            bool nshown = GUILayout.Toggle(shown, label, GUIStyles.toggleButtonStyle);
            if (shown ^ nshown)
                gui.ToggleGUI();
        }

        protected override void OnGUICustom()
        {
            if (show_while_hover)
            {
                if (!window.Contains(Mouse.screenPos))
                {
                    UnShowGUI();
                    show_while_hover = false;
                }
            }
        }

        internal void set_x_position(float x)
        {
            window.x = Math.Min(x, Screen.width - window.width);
        }

        internal void set_y_position(float y)
        {
            window.y = Mathf.Min(Mathf.Max(y - window.height / 2.0f, 0.0f), Screen.height - window.height);
            window.x = Screen.width - window.width - 42.0f;
        }
    }
}
