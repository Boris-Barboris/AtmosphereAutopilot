using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;

namespace AtmosphereAutopilot
{
    class AppLauncherWindow : GUIWindow
    {
        Dictionary<Type, object> modules;
        public AppLauncherWindow(Dictionary<Type, object> modules) :
            base("", 3920049, new Rect(Screen.width - 300, 40, 250, 30))
        {
            this.modules = modules;
        }

        public void set_left(int left)
        {
            window.x = left;
        }

        public bool show_while_hover = false;

        protected override void _drawGUI(int id)
        {
            GUILayout.BeginVertical();
            foreach (var pair in modules)
            {
                IWindow gui = pair.Value as IWindow;
                if (gui != null)
                {
                    bool shown = gui.IsShown();
                    string label = (pair.Value as GUIWindow) != null ?
                        (pair.Value as GUIWindow).WindowName :
                        pair.Key.Name;
                    bool nshown = GUILayout.Toggle(shown, label, GUIStyles.toggleButtonStyle);
                    if (shown ^ nshown)
                        gui.ToggleGUI();
                }
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
