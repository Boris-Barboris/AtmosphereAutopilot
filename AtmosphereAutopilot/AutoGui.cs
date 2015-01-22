using System;
using System.Collections.Generic;
using System.Collections;
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

        bool IsShown();

        bool ToggleGUI();

        void HideGUI();

        void UnHideGUI();

        void ShowGUI();

        void UnShowGUI();
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

        public string WindowName { get { return wndname; } }

        public bool IsShown()
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

        public void UnHideGUI()
        {
            gui_hidden = false;
        }

        public void ShowGUI()
        {
            gui_shown = true;
        }

        public void UnShowGUI()
        {
            gui_shown = false;
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

        static object[] GetCustomAttributes(object element, Type atttype, bool inherit)
        {
            PropertyInfo p = element as PropertyInfo;
            if (p != null)
                return p.GetCustomAttributes(atttype, inherit);
            FieldInfo f = element as FieldInfo;
            if (f != null)
                return f.GetCustomAttributes(atttype, inherit);
            return null;
        }

        static Type ElementType(object element)
        {
            PropertyInfo p = element as PropertyInfo;
            if (p != null)
                return p.PropertyType;
            FieldInfo f = element as FieldInfo;
            if (f != null)
                return f.FieldType;
            return null;
        }

        static object GetValue(object element, object obj)
        {
            PropertyInfo p = element as PropertyInfo;
            if (p != null)
                return p.GetValue(obj, null);
            FieldInfo f = element as FieldInfo;
            if (f != null)
                return f.GetValue(obj);
            return null;
        }

        static void SetValue(object element, object obj, object value)
        {
            PropertyInfo p = element as PropertyInfo;
            if (p != null)
                p.SetValue(obj, value, null);
            FieldInfo f = element as FieldInfo;
            if (f != null)
                f.SetValue(obj, value);
        }

        static string Name(object element)
        {
            PropertyInfo p = element as PropertyInfo;
            if (p != null)
                return p.Name;
            FieldInfo f = element as FieldInfo;
            if (f != null)
                return f.Name;
            return null;
        }

        static void draw_element(object element, object obj)
        {
            var attributes = GetCustomAttributes(element, typeof(AutoGuiAttr), true);
            if (attributes == null || attributes.Length <= 0)
                return;
            var att = attributes[0] as AutoGuiAttr;
            if (att == null)
                return;
            Type element_type = ElementType(element);
            if (element_type == null)
                return;
            if (element_type.GetInterface("IEnumarable") != null)
            {
                IEnumerable list = GetValue(element, obj) as IEnumerable;
                if (list != null)
                {
                    GUILayout.Space(5.0f);
                    foreach (object lel in list)
                        AutoDrawObject(lel);
                    GUILayout.Space(5.0f);
                }
                return;
            }
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
                if (ToStringFormat != null && att.format != null)
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
                    if (ToStringFormat != null && att.format != null)
                        val_holder = (string)ToStringFormat.Invoke(GetValue(element, obj), new[] { att.format });
                    else
                        val_holder = GetValue(element, obj).ToString();
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
