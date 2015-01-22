using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;
using System.Reflection;

namespace AtmosphereAutopilot
{
    [AttributeUsage(AttributeTargets.Property | AttributeTargets.Field, Inherited = true)]
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

        void HideGUI();

        void ShowGUI();
    }


    /// <summary>
    /// Basic window, derived class needs to implement _drawGUI method.
    /// </summary>
    abstract class GUIWindow : IAutoGui
    {
        protected string wndname;
        protected int wnd_id;
        protected bool gui_shown = false;
        protected bool gui_hidden = false;
        protected Rect window;

        public GUIWindow(string wndname, int wnd_id, Rect window)
        {
            this.wndname = wndname;
            this.wnd_id = wnd_id;
            this.window = window;
        }

        public bool IsDrawn()
        {
            return gui_shown;
        }

        public void OnGUI()
        {
            if (!gui_shown || gui_hidden)
                return;
            window = GUILayout.Window(wnd_id, window, _drawGUI, wndname);
            OnGUICustom();
        }

        protected virtual void OnGUICustom() { }

        public bool ToggleGUI()
        {
            return gui_shown = !gui_shown;
        }

        protected abstract void _drawGUI(int id);

        public void HideGUI()
        {
            gui_hidden = true;
        }

        public void ShowGUI()
        {
            gui_hidden = false;
        }
    }



    static class AutoGUI
    {
        static Dictionary<int, string> value_holders = new Dictionary<int, string>();

        public static void AutoDrawObject(object obj)
        {
            Type type = obj.GetType();
			
			// properties
            foreach (var property in type.GetProperties())
                draw_element(property, obj);

			// fields
			foreach (var field in type.GetFields())
                draw_element(field, obj);
        }

        static object[] GetCustomAttributes(object obj, Type atttype, bool inherit)
        {
            if (obj.GetType() == typeof(PropertyInfo))
                return (obj as PropertyInfo).GetCustomAttributes(atttype, inherit);
            if (obj.GetType() == typeof(FieldInfo))
                return (obj as FieldInfo).GetCustomAttributes(atttype, inherit);
            return null;
        }

        static Type ElementType(object element)
        {
            if (element.GetType() == typeof(PropertyInfo))
                return (element as PropertyInfo).PropertyType;
            if (element.GetType() == typeof(FieldInfo))
                return (element as FieldInfo).FieldType;
            return null;
        }

        static object GetValue(object element, object obj)
        {
            if (element.GetType() == typeof(PropertyInfo))
                return (element as PropertyInfo).GetValue(obj, null);
            if (element.GetType() == typeof(FieldInfo))
                return (element as FieldInfo).GetValue(obj);
            return null;
        }

        static void SetValue(object element, object obj, object value)
        {
            if (element.GetType() == typeof(PropertyInfo))
                (element as PropertyInfo).SetValue(obj, value, null);
            if (element.GetType() == typeof(FieldInfo))
                (element as FieldInfo).SetValue(obj, value);
        }

        static string Name(object element)
        {
            if (element.GetType() == typeof(PropertyInfo))
                return (element as PropertyInfo).Name;
            if (element.GetType() == typeof(FieldInfo))
                return (element as FieldInfo).Name;
            return null;
        }

        static void draw_element(object element, object obj)
        {
            var attributes = GetCustomAttributes(element, typeof(AutoGuiAttr), true);
            if (attributes.Length <= 0)
                return;
            var att = attributes[0] as AutoGuiAttr;
            if (att == null)
                return;
            Type element_type = ElementType(element);
            if (element_type == typeof(bool) && att.editable)
            {
                // it's a button
                bool cur_state = (bool)GetValue(element, obj);
                SetValue(element, obj, GUILayout.Toggle(cur_state, att.value_name + " = " + cur_state.ToString(),
                        GUIStyles.toggleButtonStyle));
                return;
            }
            GUILayout.BeginHorizontal();
            GUILayout.Label(att.value_name, GUIStyles.labelStyleLeft);
            var ToStringFormat = element_type.GetMethod("ToString", new[] { typeof(string) });
            if (!att.editable)
            {
                if (ToStringFormat != null)
                    GUILayout.Label((string)ToStringFormat.Invoke(GetValue(element, obj), new[] { att.format }), GUIStyles.labelStyleRight);
                else
                    GUILayout.Label(GetValue(element, obj).ToString(), GUIStyles.labelStyleRight);
            }
            else
            {
                int hash = obj.GetHashCode() + Name(element).GetHashCode();
                string val_holder;
                if (value_holders.ContainsKey(hash))
                    val_holder = value_holders[hash];
                else
                    if (ToStringFormat != null)
                        val_holder = (string)ToStringFormat.Invoke(GetValue(element, obj), new[] { att.format });
                    else
                        val_holder = GetValue(obj, null).ToString();
                val_holder = GUILayout.TextField(val_holder, GUIStyles.textBoxStyle);
                try
                {
                    var ParseMethod = element_type.GetMethod("Parse", new[] { typeof(string) });
                    if (ParseMethod != null)
                        SetValue(element, obj, ParseMethod.Invoke(null, new[] { val_holder }));
                }
                catch { }
                value_holders[hash] = val_holder;
            }
            GUILayout.EndHorizontal();
        }
    }
}
