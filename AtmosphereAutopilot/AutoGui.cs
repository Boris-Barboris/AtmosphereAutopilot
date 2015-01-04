using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;

namespace AtmosphereAutopilot
{
    [AttributeUsage(AttributeTargets.Property, Inherited = true)]
    class AutoGuiAttr : Attribute
    {
        public string value_name;
        public bool editable;
        public string format;
		public AutoGuiAttr(string value_name, bool editable, string format)
        {
            this.value_name = value_name;
            this.editable = editable;
            this.format = format;
        }
    }



    interface IAutoGui
    {
        void OnGUI();

        bool IsDrawn();

        bool ToggleGUI();
    }



    static class AutoGUI
    {
        static Dictionary<int, string> value_holders = new Dictionary<int, string>();

        public static void AutoDrawObject(object obj)
        {
            Type type = obj.GetType();
            foreach (var property in type.GetProperties())
            {
                var attributes = property.GetCustomAttributes(typeof(AutoGuiAttr), true);
                if (attributes.Length <= 0)
                    continue;
                var att = attributes[0] as AutoGuiAttr;
                if (att == null)
                    continue;
                Type prop_type = property.PropertyType;
                if (prop_type == typeof(bool) && att.editable)
                {
                    // it's a button
                    bool cur_state = (bool)property.GetValue(obj, null);
                    property.SetValue(obj, cur_state ^ GUILayout.Button(att.value_name + " = " + cur_state.ToString(), 
                        GUIStyles.toggleButtonStyle), null);
                    continue;
                }
                GUILayout.BeginHorizontal();
                GUILayout.Label(att.value_name, GUIStyles.labelStyle);
                var ToStringFormat = prop_type.GetMethod("ToString", new[] { typeof(string) });
                if (!att.editable)
                {
                    if (ToStringFormat != null)
                        GUILayout.Label((string)ToStringFormat.Invoke(property.GetValue(obj, null), new[] { att.format }), GUIStyles.labelStyle);
                    else
                        GUILayout.Label(property.GetValue(obj, null).ToString(), GUIStyles.labelStyle);
                }
                else
                {
                    int hash = obj.GetHashCode() + property.Name.GetHashCode();
                    string val_holder;
                    if (value_holders.ContainsKey(hash))
                        val_holder = value_holders[hash];
                    else
                        if (ToStringFormat != null)
                            val_holder = (string)ToStringFormat.Invoke(property.GetValue(obj, null), new[] { att.format });
                        else
                            val_holder = property.GetValue(obj, null).ToString();
                    val_holder = GUILayout.TextField(val_holder, GUIStyles.textBoxStyle);
                    try
                    {
                        var ParseMethod = prop_type.GetMethod("Parse", new[] { typeof(string) });
                        if (ParseMethod != null)
                            property.SetValue(obj, ParseMethod.Invoke(null, new [] { val_holder }), null);
                    }
                    catch { }
                    value_holders[hash] = val_holder;
                }
                GUILayout.EndHorizontal();
            }
        }
    }
}
