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

            labelStyleRight = new GUIStyle(skin.label);
            labelStyleRight.alignment = TextAnchor.MiddleRight;
            labelStyleRight.fontSize = 11;
            labelStyleRight.margin = new RectOffset(2, 2, 1, 1);

            labelStyleCenter = new GUIStyle(skin.label);
            labelStyleCenter.alignment = TextAnchor.MiddleCenter;
            labelStyleCenter.fontSize = 11;
            labelStyleCenter.margin = new RectOffset(2, 2, 1, 1);

            textBoxStyle = new GUIStyle(skin.textField);
            textBoxStyle.alignment = TextAnchor.MiddleCenter;
            textBoxStyle.fontSize = 11;
            textBoxStyle.margin = new RectOffset(2, 2, 1, 1);

            skin.button.active.textColor = Color.green;
            toggleButtonStyle = new GUIStyle(skin.button);
            toggleButtonStyle.alignment = TextAnchor.MiddleCenter;
            toggleButtonStyle.margin = new RectOffset(4, 4, 1, 1);
            toggleButtonStyle.fontSize = 12;
            toggleButtonStyle.stretchWidth = true;
        }
    }
}
