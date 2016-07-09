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
using System.Linq;
using System.Text;
using System.Reflection;
using UnityEngine;

namespace AtmosphereAutopilot
{
    /// <summary>
    /// Base class for auto-serializable fields and properties
    /// </summary>
    [AttributeUsage(AttributeTargets.Field | AttributeTargets.Property, Inherited = true)]
    public class AutoSerializableAttr : Attribute
    {
        public string data_name;
        public AutoSerializableAttr(string node_name)
        {
            this.data_name = node_name;
        }
    }

    /// <summary>
    /// Use this attribute to make this field auto-serializable to vessel-specific config.
    /// </summary>
    [AttributeUsage(AttributeTargets.Field | AttributeTargets.Property, Inherited = true)]
    public class VesselSerializable : AutoSerializableAttr
    {
        public VesselSerializable(string node_name) : base(node_name) { }
    }

    /// <summary>
    /// Use this attribute to make this field auto-serializable to global config.
    /// </summary>
    [AttributeUsage(AttributeTargets.Field | AttributeTargets.Property, Inherited = true)]
    public class GlobalSerializable : AutoSerializableAttr
    {
        public GlobalSerializable(string node_name) : base(node_name) { }
    }


    public interface ISerializable
    {
        bool Deserialize();

        void Serialize();
    }


    /// <summary>
    /// Automatic property and field value serialization/deserialization functionality
    /// </summary>
    public static class AutoSerialization
    {
        /// <summary>
        /// Deserialize object from file
        /// </summary>
        /// <param name="obj">Object to deserialize</param>
        /// <param name="node_name">Node to search for in file</param>
        /// <param name="filename">full file path</param>
        /// <param name="attribute_type">Type of attributes to deserialize</param>
        /// <param name="OnDeserialize">Callback for custom behaviour, 
        /// called after automatic part is over and didn't crash. Gets node, 
        /// from wich object was deserialized and attribute type.</param>
        /// <returns>true if node_name node was found and used to deserialize the object</returns>
        public static bool Deserialize(object obj, string node_name, string filename, Type attribute_type, Action<ConfigNode, Type> OnDeserialize = null)
        {
            try
            {
                if (System.IO.File.Exists(filename))
                {
                    ConfigNode node = null;
                    ConfigNode fileNode = ConfigNode.Load(filename);
                    if (fileNode != null)
                    {
                        var nodes = fileNode.GetNodes(node_name);
                        try
                        {
                            node = nodes != null ? nodes.First() : null;
                        }
                        catch { node = null; }
                        if (node != null)
                        {
                            DeserializeFromNode(node, obj, attribute_type);
                            if (OnDeserialize != null)
                                OnDeserialize(node, attribute_type);
                            return true;
                        }
                    }
                }
            }
            catch (Exception err)
            {
                Debug.Log("[AtmosphereAutopilot]: Deserialization exception in path " + filename + " message " + err.Message);
            }
            return false;
        }

        /// <summary>
        /// Serialize object to file
        /// </summary>
        /// <param name="obj">Object to serialize</param>
        /// <param name="node_name">Node to create in file</param>
        /// <param name="filename">full file path</param>
        /// <param name="attribute_type">Type of attributes to serialize</param>
        /// <param name="OnSerialize">Callback for custom behaviour, 
        /// called after automatic part is over and didn't crash. Gets node, 
        /// to wich object was serialized to and attribute type.</param>
        public static void Serialize(object obj, string node_name, string filename, Type attribute_type,
            Action<ConfigNode, Type> OnSerialize = null)
        {
            try
            {
                string dir = System.IO.Path.GetDirectoryName(filename);
                if (!System.IO.Directory.Exists(dir))
                    System.IO.Directory.CreateDirectory(dir);
                ConfigNode fileNode = ConfigNode.Load(filename);
                if (fileNode == null)
                    fileNode = new ConfigNode();
                else
                    fileNode.RemoveNode(node_name);
                ConfigNode node = new ConfigNode(node_name);
                SerializeToNode(node, obj, attribute_type);
                if (OnSerialize != null)
                    OnSerialize(node, attribute_type);
                if (node.HasData)
                {
                    fileNode.AddNode(node);
                    fileNode.Save(filename);
                }
            }
            catch (Exception err)
            {
                Debug.Log("[AtmosphereAutopilot]: Serialization exception in path " + filename + " message " + err.Message);
            }
        }

        public static void DeserializeFromNode(ConfigNode node, object obj, Type attribute_type)
        {
            Type type = obj.GetType();
            foreach (var field in type.GetFields(BindingFlags.Instance | BindingFlags.Static | BindingFlags.Public | BindingFlags.NonPublic))
            {
                var attributes = field.GetCustomAttributes(attribute_type, true);
                if (attributes.Length <= 0)
                    continue;
                var att = attributes[0] as AutoSerializableAttr;
                if (att == null)
                    continue;
                string str = node.GetValue(att.data_name);
                if (str == null)
                    continue;
                if (field.FieldType == typeof(string))
                {
                    field.SetValue(obj, str);
                    continue;
                }
                if (field.FieldType.IsEnum)
                    field.SetValue(obj, Enum.Parse(field.FieldType, str));
                else
                {
                    var parse_method = field.FieldType.GetMethod("Parse", new[] { typeof(string) });
                    if (parse_method == null)
                        continue;
                    field.SetValue(obj, parse_method.Invoke(null, new[] { str }));
                }
            }
            foreach (var property in type.GetProperties(BindingFlags.Instance | BindingFlags.Static | BindingFlags.Public | BindingFlags.NonPublic))
            {
                var attributes = property.GetCustomAttributes(attribute_type, true);
                if (attributes.Length <= 0)
                    continue;
                var att = attributes[0] as AutoSerializableAttr;
                if (att == null)
                    continue;
                string str = node.GetValue(att.data_name);
                if (str == null)
                    continue;
                if (property.PropertyType == typeof(string))
                {
                    property.SetValue(obj, str, null);
                    continue;
                }
                if (property.PropertyType.IsEnum)
                    property.SetValue(obj, Enum.Parse(property.PropertyType, str), null);
                else
                {
                    var parse_method = property.PropertyType.GetMethod("Parse", new[] { typeof(string) });
                    if (parse_method == null)
                        continue;
                    property.SetValue(obj, parse_method.Invoke(null, new[] { str }), null);
                }
            }
        }

        public static void SerializeToNode(ConfigNode node, object obj, Type attribute_type)
        {
            // Serialize
            Type type = obj.GetType();
            foreach (var field in type.GetFields(BindingFlags.Instance | BindingFlags.Static | BindingFlags.Public | BindingFlags.NonPublic))
            {
                var attributes = field.GetCustomAttributes(attribute_type, true);
                if (attributes.Length <= 0)
                    continue;
                var att = attributes[0] as AutoSerializableAttr;
                if (att == null)
                    continue;
                string str = field.GetValue(obj).ToString();
                if (str == null)
                    continue;
                node.AddValue(att.data_name, str);
            }
            foreach (var property in type.GetProperties(BindingFlags.Instance | BindingFlags.Static | BindingFlags.Public | BindingFlags.NonPublic))
            {
                var attributes = property.GetCustomAttributes(attribute_type, true);
                if (attributes.Length <= 0)
                    continue;
                var att = attributes[0] as AutoSerializableAttr;
                if (att == null)
                    continue;
                string str = property.GetValue(obj, null).ToString();
                if (str == null)
                    continue;
                node.AddValue(att.data_name, str);
            }
        }
    }
}
