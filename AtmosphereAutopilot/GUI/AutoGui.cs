/*
Atmosphere Autopilot, plugin for Kerbal Space Program.
Copyright (C) 2015-2016, Baranin Alexander aka Boris-Barboris.
 
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

using System;
using System.Collections.Generic;
using System.Collections;
using System.Linq;
using System.Text;
using UnityEngine;
using System.Reflection;

namespace AtmosphereAutopilot
{
    /// <summary>
    /// Attribute for auto-rendered parameters. Use it on property or field to draw it
    /// by AutoGUI.AutoDrawObject method. Supports all basic types and IEnumarable.
    /// </summary>
    [AttributeUsage(AttributeTargets.Property | AttributeTargets.Field, Inherited = true)]
    public class AutoGuiAttr : Attribute
    {
        internal string value_name;
        internal bool editable;
        internal string format;
        internal object[] format_arr;

        /// <summary>
        /// Set this property or field as auto-renderable.
        /// </summary>
        /// <param name="value_name">Displayed element name</param>
        /// <param name="editable">Can be edited by user. Use for basic types only!</param>
        /// <param name="format">If type provides ToString(string format) method, this format string
        /// will be used. You can set it to null if not required</param>
        public AutoGuiAttr(string value_name, bool editable, string format = null)
        {
            this.value_name = value_name;
            this.editable = editable;
            this.format = format;
            format_arr = new object[] { this.format };
        }
    }


    /// <summary>
    /// Interface for all windows.
    /// </summary>
    public interface IWindow
    {
        /// <summary>
        /// OnGUI Unity event handler
        /// </summary>
        void OnGUI();

        /// <summary>
        /// Returns true if window is shown.
        /// </summary>
        bool IsShown();

        /// <summary>
        /// Toggle window shown\unshown state
        /// </summary>
        bool ToggleGUI();

        /// <summary>
        /// Show window.
        /// </summary>
        void ShowGUI();

        /// <summary>
        /// Do not show window.
        /// </summary>
        void UnShowGUI();
    }


    /// <summary>
    /// Basic window, derived class needs to implement _drawGUI method.
    /// </summary>
    public abstract class GUIWindow : IWindow
    {
        string wndname;
        int wnd_id;
        bool gui_shown = false;
        protected Rect window;

        /// <summary>
        /// Create window instance.
        /// </summary>
        /// <param name="wndname">Window header</param>
        /// <param name="wnd_id">Unique for Unity engine id</param>
        /// <param name="window">Initial window position rectangle</param>
        internal GUIWindow(string wndname, int wnd_id, Rect window)
        {
            this.wndname = wndname;
            this.wnd_id = wnd_id;
            this.window = window;
        }

        /// <summary>
        /// Get window header.
        /// </summary>
        public string WindowName { get { return wndname; } }

        /// <inheritdoc />
        public bool IsShown()
        {
            return gui_shown;
        }

        /// <inheritdoc />
        public void OnGUI()
        {
            OnGUICustomAlways();
            if (!gui_shown || AtmosphereAutopilot.UIHidden)
                return;
            
            // forbid windows not on screen
            window.xMin = Common.Clampf(window.xMin, 5.0f - window.width, Screen.width - 5.0f);
            window.yMin = Common.Clampf(window.yMin, 5.0f - window.height, Screen.height - 5.0f);

            window = GUILayout.Window(wnd_id, window, _drawGUI, wndname);
            OnGUICustom();
        }

        /// <summary>
        /// Procedure for close "x" button in upper right corner
        /// </summary>
        protected void close_button()
        {
            Rect close_btn_rect = new Rect(window.width - 16.0f, 1.0f, 15.0f, 16.0f);
            bool close = GUI.Button(close_btn_rect, "x", GUIStyles.toggleButtonStyle);
            if (close)
                this.UnShowGUI();
        }

        /// <summary>
        /// Called after each _drawGUI call
        /// </summary>
        protected virtual void OnGUICustom() { }

        /// <summary>
        // Executed on every OnGUI regardless of gui_shown or UIHidden
        /// </summary>
        protected virtual void OnGUICustomAlways() { }

        /// <inheritdoc />
        public bool ToggleGUI()
        {
            return gui_shown = !gui_shown;
        }

        /// <summary>
        /// Main drawing function
        /// </summary>
        /// <param name="id">Unique window id. Just ignore it in function realization.</param>
        protected abstract void _drawGUI(int id);

        /// <inheritdoc />
        public virtual void ShowGUI()
        {
            gui_shown = true;
        }

        /// <inheritdoc />
        public virtual void UnShowGUI()
        {
            gui_shown = false;
        }
    }


    /// <summary>
    /// Automatic property and field rendering functionality.
    /// </summary>
    public static class AutoGUI
    {
        // optimization structures
        static Dictionary<Type, PropertyInfo[]> property_list = new Dictionary<Type, PropertyInfo[]>();
        static Dictionary<Type, FieldInfo[]> field_list = new Dictionary<Type, FieldInfo[]>();
        static Dictionary<object, object[]> custom_attrs = new Dictionary<object, object[]>();
        static Dictionary<Type, MethodInfo> toStringMethods = new Dictionary<Type, MethodInfo>();
        static Dictionary<Type, MethodInfo> parseMethods = new Dictionary<Type, MethodInfo>();
        static readonly Type[] formatStrTypes = { typeof(string) };

        /// <summary>
        /// Render class instace using AutoGuiAttr markup.
        /// </summary>
        /// <param name="obj">object to render to current GUILayout.</param>
        public static void AutoDrawObject(object obj)
        {
            Type type = obj.GetType();

            if (type.IsPrimitive)
            {
                draw_primitive(obj);
                return;
            }
            
            // properties
            if (!property_list.ContainsKey(type))
            {
                List<PropertyInfo> properties = new List<PropertyInfo>(type.GetProperties(BindingFlags.Instance |
                    BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Static));
                properties.RemoveAll(pi => pi.GetCustomAttributes(typeof(AutoGuiAttr), true).Length == 0);
                property_list[type] = properties.ToArray();
            }
            foreach (var property in property_list[type])
                draw_element(property, obj);

            // fields
            if (!field_list.ContainsKey(type))
            {
                List<FieldInfo> fields = new List<FieldInfo>(type.GetFields(BindingFlags.Instance |
                    BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Static));
                fields.RemoveAll(fi => fi.GetCustomAttributes(typeof(AutoGuiAttr), true).Length == 0);
                field_list[type] = fields.ToArray();
            }
            foreach (var field in field_list[type])
                draw_element(field, obj);
        }


        #region FieldPropertyUniversal

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

        #endregion


        static void draw_primitive(object obj)
        {
            GUILayout.Label(obj.ToString(), GUIStyles.labelStyleRight);
        }

        /// <summary>
        /// Main rendering function.
        /// </summary>
        /// <param name="element">Field or property info to render</param>
        /// <param name="obj">Object instance</param>
        static void draw_element(object element, object obj)
        {
            if (!custom_attrs.ContainsKey(element))
                custom_attrs[element] = GetCustomAttributes(element, typeof(AutoGuiAttr), true);
            object[] attributes = custom_attrs[element];
            if (attributes == null || attributes.Length <= 0)
                return;
            var att = attributes[0] as AutoGuiAttr;
            if (att == null)
                return;
            Type element_type = ElementType(element);
            if (element_type == null)
                return;

            // If element is collection
            if (typeof(IEnumerable).IsAssignableFrom(element_type))
            {
                IEnumerable list = GetValue(element, obj) as IEnumerable;
                if (list != null)
                {
                    GUILayout.BeginHorizontal();
                    GUILayout.Label(att.value_name + ':', GUIStyles.labelStyleLeft);
                    GUILayout.BeginVertical();
                    foreach (object lel in list)
                        AutoDrawObject(lel);        // render each member
                    GUILayout.EndVertical();
                    GUILayout.EndHorizontal();
                    return;
                }
            }

            if (element_type == typeof(bool) && att.editable)
            {
                // it's a toggle button
                bool cur_state = (bool)GetValue(element, obj);
                SetValue(element, obj, GUILayout.Toggle(cur_state, att.value_name,
                        GUIStyles.toggleButtonStyle));
                return;
            }

            if (element_type == typeof(string))
            {
                // it's a toggle button
                GUILayout.Label(att.value_name + ": " + (string)GetValue(element, obj), GUIStyles.labelStyleCenter);
                return;
            }

            GUILayout.BeginHorizontal();
            GUILayout.Label(att.value_name, GUIStyles.labelStyleLeft);

            if (element_type == typeof(Vector3d) && !att.editable && att.format != null)
            {
                GUILayout.Label(((Vector3d)GetValue(element, obj)).ToString(att.format), GUIStyles.labelStyleRight);
                GUILayout.EndHorizontal();
                return;
            }

            if (!toStringMethods.ContainsKey(element_type))
                toStringMethods[element_type] = element_type.GetMethod("ToString", formatStrTypes);
            MethodInfo ToStringFormat = toStringMethods[element_type];
            if (!att.editable)
            {
                if (ToStringFormat != null && att.format != null)
                    GUILayout.Label((string)ToStringFormat.Invoke(GetValue(element, obj), att.format_arr), GUIStyles.labelStyleRight);
                else
                    GUILayout.Label(GetValue(element, obj).ToString(), GUIStyles.labelStyleRight);
            }
            else
            {
                string val_holder;
                if (ToStringFormat != null && att.format != null)
                    val_holder = (string)ToStringFormat.Invoke(GetValue(element, obj), att.format_arr);
                else
                    val_holder = GetValue(element, obj).ToString();
                val_holder = GUILayout.TextField(val_holder, GUIStyles.textBoxStyle);
                try
                {
                    if (!parseMethods.ContainsKey(element_type))
                        parseMethods[element_type] = element_type.GetMethod("Parse", formatStrTypes);
                    var ParseMethod = parseMethods[element_type];
                    if (ParseMethod != null)
                        SetValue(element, obj, ParseMethod.Invoke(null, new[] { val_holder }));
                }
                catch { }
            }
            GUILayout.EndHorizontal();
        }
    }
}
