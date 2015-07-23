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
using UnityEngine;

namespace AtmosphereAutopilot
{
    class AppLauncherWindow : GUIWindow
    {
		public AppLauncherWindow() :
			base("", 3920049, new Rect(Screen.width - 260, 38, 250, 30))
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
                IWindow gui = pair.Value;
                bool shown = gui.IsShown();
                string label = (pair.Value as GUIWindow) != null ?
                    (pair.Value as GUIWindow).WindowName :
                    pair.Key.Name;
                bool nshown = GUILayout.Toggle(shown, label, GUIStyles.toggleButtonStyle);
                if (shown ^ nshown)
                    gui.ToggleGUI();
            }
            GUILayout.EndVertical();
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
    }
}
