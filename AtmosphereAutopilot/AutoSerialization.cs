using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace AtmosphereAutopilot
{
    [AttributeUsage(AttributeTargets.Field | AttributeTargets.Property, Inherited = true)]
    class AutoSerializableAttr : Attribute
    {
        public string data_name;
        public AutoSerializableAttr(string data_name)
        {
            this.data_name = data_name;
        }
    }

    [AttributeUsage(AttributeTargets.Field | AttributeTargets.Property, Inherited = true)]
    class VesselSerializable : AutoSerializableAttr
    {
        public VesselSerializable(string data_name) : base(data_name) { }
    }

    [AttributeUsage(AttributeTargets.Field | AttributeTargets.Property, Inherited = true)]
    class GlobalSerializable : AutoSerializableAttr
    {
        public GlobalSerializable(string data_name) : base(data_name) { }
    }


    interface IAutoSerializable
    {
        bool Deserialize();

        void Serialize();
    }


    public static class AutoSerialization
    {
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

        public static void Serialize(object obj, string node_name, string filename, Type attribute_type, Action<ConfigNode, Type> OnSerialize = null)
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

        public static void DeserializeFromNode(ConfigNode node, object obj, Type attribute_type)
        {
            Type type = obj.GetType();
            foreach (var field in type.GetFields())
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
            foreach (var property in type.GetProperties())
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

        public static void SerializeToNode(ConfigNode node, object obj, Type attribute_type)
        {
            // Serialize
            Type type = obj.GetType();
            foreach (var field in type.GetFields())
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
            foreach (var property in type.GetProperties())
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
