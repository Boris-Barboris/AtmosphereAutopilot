using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;

namespace AtmosphereAutopilot
{
    static class GUIStyles
    {
        public static GUISkin skin;
        public static GUIStyle labelStyleLeft;
        public static GUIStyle labelStyleCenter;
        public static GUIStyle labelStyleRight;
        public static GUIStyle textBoxStyle;
        public static GUIStyle toggleButtonStyle;

        public static void Init()
        {
            skin = GUI.skin;

            labelStyleLeft = new GUIStyle(GUI.skin.label);
            labelStyleLeft.alignment = TextAnchor.MiddleLeft;
            labelStyleLeft.fontSize = 11;
            labelStyleLeft.margin = new RectOffset(2, 2, 1, 1);

            labelStyleRight = new GUIStyle(GUI.skin.label);
            labelStyleRight.alignment = TextAnchor.MiddleRight;
            labelStyleRight.fontSize = 11;
            labelStyleRight.margin = new RectOffset(2, 2, 1, 1);

            labelStyleCenter = new GUIStyle(GUI.skin.label);
            labelStyleCenter.alignment = TextAnchor.MiddleCenter;
            labelStyleCenter.fontSize = 11;
            labelStyleCenter.margin = new RectOffset(2, 2, 1, 1);

            textBoxStyle = new GUIStyle(GUI.skin.textField);
            textBoxStyle.alignment = TextAnchor.MiddleCenter;
            textBoxStyle.fontSize = 11;
            textBoxStyle.margin = new RectOffset(2, 2, 1, 1);

            toggleButtonStyle = new GUIStyle(GUI.skin.button);
            toggleButtonStyle.alignment = TextAnchor.MiddleCenter;
            toggleButtonStyle.margin = new RectOffset(4, 4, 1, 1);
            toggleButtonStyle.fontSize = 12;
            toggleButtonStyle.stretchWidth = true;
        }
    }
}
