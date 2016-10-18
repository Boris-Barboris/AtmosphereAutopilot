using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;
using System.Reflection;

namespace AtmosphereAutopilot
{

    /// <summary>
    /// Attribute to mark auto-managed hotkeys
    /// </summary>
    [AttributeUsage(AttributeTargets.Field, Inherited = true, AllowMultiple = false)]
    public class AutoHotkeyAttr : Attribute
    {
        /// <summary>
        /// Name of the hotkey to display
        /// </summary>
        public string hotkey_name;

        public AutoHotkeyAttr(string hotkey_name)
        {
            this.hotkey_name = hotkey_name;
        }
    }

    /// <summary>
    /// Window to manage hotkeys for different modules
    /// </summary>
    public sealed class AutoHotkey : GUIWindow, ISerializable
    {
        struct HotkeyField
        {
            public AutoHotkeyAttr attr;
            public FieldInfo field;

            public HotkeyField(FieldInfo field, AutoHotkeyAttr attr)
            {
                this.field = field;
                this.attr = attr;
            }
        }

        List<HotkeyField> hotkey_map = new List<HotkeyField>();

        public AutoHotkey(List<Type> modules_list) : base("Hotkeys manager", 3920050, new Rect(Screen.width - 280, 38, 230, 30))
        {
            // let's create the mapping
            foreach (var module_type in modules_list)
            {
                foreach (var field in module_type.GetFields(BindingFlags.Static | BindingFlags.Public | BindingFlags.NonPublic))
                {
                    var attrs = (AutoHotkeyAttr[])field.GetCustomAttributes(typeof(AutoHotkeyAttr), true);
                    if (attrs == null || attrs.Length < 1)
                        continue;
                    hotkey_map.Add(new HotkeyField(field, attrs[0]));
                }
            }
        }

        int editing_index = -1;

        protected override void _drawGUI(int id)
        {
            close_button();
            GUILayout.BeginVertical();

            int new_editing_index = editing_index;
            for (int i = 0; i < hotkey_map.Count; i++)
            {
                GUILayout.BeginHorizontal();

                FieldInfo field = hotkey_map[i].field;
                AutoHotkeyAttr attr = hotkey_map[i].attr;
                GUILayout.Label(attr.hotkey_name, GUIStyles.labelStyleLeft);
                string text = (editing_index == i) ? "??" : field.GetValue(null).ToString();
                bool pressed = GUILayout.Button(text, GUIStyles.toggleButtonStyle, GUILayout.MinWidth(75.0f), GUILayout.Width(75.0f));
                if (pressed)
                    new_editing_index = i;

                GUILayout.EndHorizontal();
            }

            GUILayout.EndVertical();
            GUI.DragWindow();

            // now perform input binding
            editing_index = new_editing_index;
            if (editing_index > -1)
            {
                if (Input.anyKeyDown)
                {
                    if (Input.GetKeyDown(KeyCode.Escape))
                    {
                        FieldInfo field = hotkey_map[editing_index].field;
                        field.SetValue(null, KeyCode.None);
                        editing_index = -1;
                    }
                    else
                    {
                        foreach (KeyCode vKey in Enum.GetValues(typeof(KeyCode)))
                        {
                            if (Input.GetKey(vKey))
                            {
                                FieldInfo field = hotkey_map[editing_index].field;
                                field.SetValue(null, vKey);
                                editing_index = -1;
                                break;
                            }
                        }
                    }
                }
            }
        }

        [GlobalSerializable("window_x")]
        public float WindowLeft
        {
            get { return window.xMin; }
            set
            {
                float width = window.width;
                window.xMin = value;
                window.xMax = window.xMin + width;
            }
        }

        [GlobalSerializable("window_y")]
        public float WindowTop
        {
            get { return window.yMin; }
            set
            {
                float height = window.height;
                window.yMin = value;
                window.yMax = window.yMin + height;
            }
        }

        /// <summary>
        /// Serialize window position
        /// </summary>
        public void Serialize()
        {
            AutoSerialization.Serialize(this, "hotkey_manager",
                KSPUtil.ApplicationRootPath + "GameData/AtmosphereAutopilot/Global_settings.txt",
                typeof(GlobalSerializable));
        }

        /// <summary>
        /// Deserialize window position
        /// </summary>
        public bool Deserialize()
        {
            return AutoSerialization.Deserialize(this, "hotkey_manager",
                KSPUtil.ApplicationRootPath + "GameData/AtmosphereAutopilot/Global_settings.txt",
                typeof(GlobalSerializable));
        }

    }
}
