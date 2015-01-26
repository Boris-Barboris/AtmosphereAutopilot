using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Reflection;

namespace AtmosphereAutopilot
{
	/// <summary>
	/// Base class for auto-serializable fields and properties
	/// </summary>
    [AttributeUsage(AttributeTargets.Field | AttributeTargets.Property, Inherited = true)]
    public class AutoSerializableAttr : Attribute
    {
        public string data_name;
        public AutoSerializableAttr(string data_name)
        {
            this.data_name = data_name;
        }
    }

	/// <summary>
	/// Use this attribute to make this field auto-serializable to vessel-specific config.
	/// </summary>
    [AttributeUsage(AttributeTargets.Field | AttributeTargets.Property, Inherited = true)]
    public class VesselSerializable : AutoSerializableAttr
    {
        public VesselSerializable(string data_name) : base(data_name) { }
    }

	/// <summary>
	/// Use this attribute to make this field auto-serializable to global config.
	/// </summary>
    [AttributeUsage(AttributeTargets.Field | AttributeTargets.Property, Inherited = true)]
    public class GlobalSerializable : AutoSerializableAttr
    {
        public GlobalSerializable(string data_name) : base(data_name) { }
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
		/// <returns>true if deserialization was not a failure</returns>
        public static bool Deserialize(object obj, string node_name, string filename, Type attribute_type, Action<ConfigNode, Type> OnDeserialize = null)
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
            ConfigNode fileNode = ConfigNode.Load(filename);
            if (fileNode == null)
                fileNode = new ConfigNode();
            fileNode.RemoveNode(node_name);
            ConfigNode node = new ConfigNode(node_name);
            SerializeToNode(node, obj, attribute_type);
            if (OnSerialize != null)
                OnSerialize(node, attribute_type);
            fileNode.AddNode(node);
            fileNode.Save(filename);
        }

        static void DeserializeFromNode(ConfigNode node, object obj, Type attribute_type)
        {
            Type type = obj.GetType();
            foreach (var field in type.GetFields(BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic))
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
                var parse_method = field.FieldType.GetMethod("Parse", new [] { typeof(string) });
                if (parse_method == null)
                    continue;
                field.SetValue(obj, parse_method.Invoke(null, new[] { str }));
            }
            foreach (var property in type.GetProperties(BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic))
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
                var parse_method = property.PropertyType.GetMethod("Parse", new[] { typeof(string) });
                if (parse_method == null)
                    continue;
                property.SetValue(obj, parse_method.Invoke(null, new[] { str }), null);
            }
        }

        static void SerializeToNode(ConfigNode node, object obj, Type attribute_type)
        {
            // Serialize
            Type type = obj.GetType();
            foreach (var field in type.GetFields(BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic))
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
            foreach (var property in type.GetProperties(BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic))
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
