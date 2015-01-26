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
			base("", 3920049, new Rect(Screen.width - 300, 40, 250, 30))
		{ }

        public void set_left(int left)
        {
            window.x = left;
        }

        public bool show_while_hover = false;

        protected override void _drawGUI(int id)
        {
            GUILayout.BeginVertical();

			foreach (var pair in AtmosphereAutopilot.Instance.getCurVesselModules())
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
