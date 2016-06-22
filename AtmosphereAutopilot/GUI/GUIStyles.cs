/*
Atmosphere Autopilot, plugin for Kerbal Space Program.
Copyright (C) 2015-2016, Baranin Alexander aka Boris-Barboris.
Copyright (C) 2016, George Sedov.
 
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

using UnityEngine;
using UnityEngine.UI;

namespace AtmosphereAutopilot
{
    /// <summary>
    /// Collection of standard for AtmosphereAutopilot AutoGUI styles
    /// </summary>
    public static class GUIStyles
    {
        public static GUISkin skin { get; private set; }
        public static GUIStyle labelStyleLeft { get; private set; }
        public static GUIStyle labelStyleCenter { get; private set; }
        public static GUIStyle labelStyleRight { get; private set; }
        public static GUIStyle textBoxStyle { get; private set; }
        public static GUIStyle toggleButtonStyle { get; private set; }
        public static GUIStyle toggleButtonStyleRight { get; private set; }
        public static GUIStyle hoverLabel { get; private set; }

        private static UISkinDef KSPSkin = UISkinManager.defaultSkin;

        internal static void Init()
        {
            skin = GUI.skin;

            labelStyleLeft = new GUIStyle(skin.label);
            labelStyleLeft.alignment = TextAnchor.MiddleLeft;
            labelStyleLeft.fontSize = 11;
            labelStyleLeft.margin = new RectOffset(2, 2, 1, 1);
            labelStyleLeft.padding = new RectOffset(1, 1, 1, 1);

            labelStyleRight = new GUIStyle(skin.label);
            labelStyleRight.alignment = TextAnchor.MiddleRight;
            labelStyleRight.fontSize = 11;
            labelStyleRight.margin = new RectOffset(2, 2, 1, 1);
            labelStyleRight.padding = new RectOffset(1, 1, 1, 1);

            labelStyleCenter = new GUIStyle(skin.label);
            labelStyleCenter.alignment = TextAnchor.MiddleCenter;
            labelStyleCenter.fontSize = 11;
            labelStyleCenter.margin = new RectOffset(2, 2, 1, 1);
            labelStyleCenter.padding = new RectOffset(1, 1, 1, 1);

            hoverLabel = new GUIStyle(skin.label);
            hoverLabel.alignment = TextAnchor.MiddleCenter;
            hoverLabel.fontSize = 14;
            hoverLabel.fontStyle = FontStyle.Bold;
            hoverLabel.normal.textColor = Color.red;
            hoverLabel.margin = new RectOffset(2, 2, 1, 1);
            hoverLabel.padding = new RectOffset(1, 1, 1, 1);
            hoverLabel.stretchHeight = true;
            hoverLabel.stretchWidth = true;

            textBoxStyle = new GUIStyle(skin.textField);
            textBoxStyle.alignment = TextAnchor.MiddleCenter;
            textBoxStyle.fontSize = 11;
            textBoxStyle.margin = new RectOffset(2, 2, 1, 1);

            toggleButtonStyle = new GUIStyle(skin.button);
            toggleButtonStyle.alignment = TextAnchor.MiddleCenter;
            toggleButtonStyle.margin = new RectOffset(4, 4, 1, 1);
            toggleButtonStyle.padding = new RectOffset(4, 4, 2, 2);
            var button_pressed_style = toggleButtonStyle.onActive;
            button_pressed_style.textColor = Color.green;
            toggleButtonStyle.normal.textColor = Color.white;
            toggleButtonStyle.fontSize = 12;
            toggleButtonStyle.onNormal = button_pressed_style;
            toggleButtonStyle.stretchWidth = true;

            toggleButtonStyleRight = new GUIStyle(skin.button);
            toggleButtonStyleRight.alignment = TextAnchor.MiddleRight;
            toggleButtonStyleRight.margin = new RectOffset(4, 4, 1, 1);
            toggleButtonStyleRight.padding = new RectOffset(4, 4, 2, 2);
            toggleButtonStyleRight.onNormal = button_pressed_style;
            toggleButtonStyleRight.fontSize = 12;
            toggleButtonStyleRight.stretchWidth = true;
        }

        static Color old_background, old_color, old_content;

        static Color my_background = new Color(0.0f, 0.0f, 0.0f, 1.7f);
        static Color my_color = new Color(1.0f, 1.0f, 1.0f, 0.5f);
        static Color my_content = new Color(1.0f, 0.2f, 0.2f, 2.0f);

        internal static void set_colors()
        {
            old_background = GUI.backgroundColor;
            old_color = GUI.color;
            old_content = GUI.contentColor;
            GUI.backgroundColor = my_background;
            GUI.color = my_color;
            GUI.contentColor = my_content;
        }

        internal static void reset_colors()
        {
            GUI.backgroundColor = old_background;
            GUI.color = old_color;
            GUI.contentColor = old_content;
        }


        internal static void Process (GameObject gameObject) {
            if (gameObject == null)
                return;

            foreach (var applicator in gameObject.GetComponentsInChildren<UI.StyleApplicator> (true))
                Process (applicator);
        }

        private static void Process (UI.StyleApplicator applicator) {
            switch (applicator.ElementType) {
                case UI.StyleApplicator.ElementTypes.Window:
                  applicator.SetImage (KSPSkin.window.normal.background, Image.Type.Sliced);
                  break;
                case UI.StyleApplicator.ElementTypes.Button:
                  applicator.SetSelectable (KSPSkin.button.normal.background,
                                            KSPSkin.button.highlight.background,
                                            KSPSkin.button.active.background,
                                            KSPSkin.button.disabled.background);
                  break;
                case UI.StyleApplicator.ElementTypes.ButtonToggle:
                  applicator.SetToggle (KSPSkin.button.normal.background,
                                        KSPSkin.button.highlight.background,
                                        KSPSkin.button.active.background,
                                        KSPSkin.button.disabled.background);
                  break;
                case UI.StyleApplicator.ElementTypes.Slider:
                  applicator.SetSlider (KSPSkin.horizontalSliderThumb.normal.background,
                                        KSPSkin.horizontalSliderThumb.highlight.background,
                                        KSPSkin.horizontalSliderThumb.active.background,
                                        KSPSkin.horizontalSliderThumb.disabled.background,
                                        KSPSkin.horizontalSlider.normal.background);
                  break;
            }
        }
    }
}
