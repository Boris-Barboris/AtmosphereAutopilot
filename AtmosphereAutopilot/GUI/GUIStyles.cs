using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;

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

        internal static void Init()
        {
            skin = GUI.skin;

            labelStyleLeft = new GUIStyle(skin.label);
            labelStyleLeft.alignment = TextAnchor.MiddleLeft;
            labelStyleLeft.fontSize = 11;
            labelStyleLeft.margin = new RectOffset(2, 2, 1, 1);
			labelStyleLeft.font.material.color = font_color;

            labelStyleRight = new GUIStyle(skin.label);
            labelStyleRight.alignment = TextAnchor.MiddleRight;
            labelStyleRight.fontSize = 11;
            labelStyleRight.margin = new RectOffset(2, 2, 1, 1);
			labelStyleRight.font.material.color = font_color;

            labelStyleCenter = new GUIStyle(skin.label);
            labelStyleCenter.alignment = TextAnchor.MiddleCenter;
            labelStyleCenter.fontSize = 11;
            labelStyleCenter.margin = new RectOffset(2, 2, 1, 1);
			labelStyleCenter.font.material.color = font_color;

            textBoxStyle = new GUIStyle(skin.textField);
            textBoxStyle.alignment = TextAnchor.MiddleCenter;
            textBoxStyle.fontSize = 11;
            textBoxStyle.margin = new RectOffset(2, 2, 1, 1);
			textBoxStyle.font.material.color = font_color;

            toggleButtonStyle = new GUIStyle(skin.button);
            toggleButtonStyle.alignment = TextAnchor.MiddleCenter;
            toggleButtonStyle.margin = new RectOffset(4, 4, 1, 1);
            toggleButtonStyle.fontSize = 12;
            toggleButtonStyle.stretchWidth = true;
			toggleButtonStyle.font.material.color = font_color;
        }

        static Color old_background, old_color, old_content;

        static Color my_background = new Color(0.0f, 0.0f, 0.0f, 2.0f);
        static Color my_color = new Color(1.0f, 1.0f, 1.0f, 0.3f);
        static Color my_content = new Color(1.0f, 0.2f, 0.2f, 3.3f);
		static Color font_color = new Color(1.0f, 0.2f, 0.2f, 1.0f);

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
    }
}
