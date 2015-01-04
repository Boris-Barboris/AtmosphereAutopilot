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
        public Action custom_drawer = null;
        public AutoGuiAttr(string value_name, bool editable, string format, Action drawFunction = null)
        {
            this.value_name = value_name;
            this.editable = editable;
            this.format = format;
            this.custom_drawer = drawFunction;
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
        static Dictionary<string, string> value_holders = new Dictionary<string, string>();

        public static void AutoDrawObject(object obj)
        {
            Type type = obj.GetType();
            foreach (var property in type.GetProperties())
            {
                var attributes = property.GetCustomAttributes(typeof(AutoGuiAttr), true);
                if (attributes == null)
                    continue;
                var att = attributes.First() as AutoGuiAttr;
                if (att == null)
                    continue;
                if (att.custom_drawer != null)
                {
                    att.custom_drawer();
                    continue;
                }
                GUILayout.BeginHorizontal();
                GUILayout.Label(att.value_name, GUIStyles.labelStyle);
                var ToStringFormat = type.GetMethod("ToString", new[] { typeof(string) });
                if (!att.editable)
                {
                    if (ToStringFormat != null)
                        GUILayout.Label((string)ToStringFormat.Invoke(property.GetValue(obj, null), new[] { att.format }), GUIStyles.labelStyle);
                    else
                        GUILayout.Label(property.GetValue(obj, null).ToString(), GUIStyles.labelStyle);
                }
                else
                {
                    string hash_str = obj.GetHashCode().ToString() + property.ToString();
                    string val_holder;
                    if (value_holders.ContainsKey(hash_str))
                        val_holder = value_holders[hash_str];
                    else
                        val_holder = "0.0";
                    val_holder = GUILayout.TextField(val_holder, GUIStyles.textBoxStyle);
                    try
                    {
                        var ParseMethod = property.GetType().GetMethod("Parse");
                        if (ParseMethod != null)
                            property.SetValue(obj, ParseMethod.Invoke(null, new [] { val_holder }), null);
                    }
                    catch { }
                    value_holders[hash_str] = val_holder;
                }
                GUILayout.EndHorizontal();
            }
        }
    }
}
