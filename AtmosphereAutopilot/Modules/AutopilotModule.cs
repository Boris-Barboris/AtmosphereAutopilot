using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Text;
using UnityEngine;

namespace AtmosphereAutopilot
{

    abstract class AutopilotModule : IAutoSerializable, IAutoGui
    {
        protected Vessel vessel = null;
        protected bool enabled = false;

        public bool Active { get { return enabled; } }

        public AutopilotModule(Vessel v, int wnd_id, string module_name)
        {
            vessel = v;
            this.module_name = module_name;
            this.wnd_id = wnd_id;
        }

        public void Activate()
        {
            if (!enabled)
                OnActivate();
            enabled = true;
        }

        protected abstract void OnActivate();

        public void Deactivate()
        {
            if (enabled)
                OnDeactivate();
            enabled = false;
        }

        protected abstract void OnDeactivate();



        #region Serialization

        public bool DeserializeVesselSpecific()
        {
            return Deserialize(module_name.Replace(' ', '_'),
                KSPUtil.ApplicationRootPath + "GameData/AtmosphereAutopilot/" + vessel.vesselName + ".cfg",
                typeof(VesselSerializable));
        }

        public bool DeserializeGlobalSpecific()
        {
            return Deserialize(module_name.Replace(' ', '_'), 
                KSPUtil.ApplicationRootPath + "GameData/AtmosphereAutopilot/Global_settings.cfg",
                typeof(GlobalSerializable));
        }

        public void SerializeAll()
        {
            Serialize(module_name.Replace(' ', '_'),
                KSPUtil.ApplicationRootPath + "GameData/AtmosphereAutopilot/" + vessel.vesselName + ".cfg",
                typeof(VesselSerializable));
            Serialize(module_name.Replace(' ', '_'),
                KSPUtil.ApplicationRootPath + "GameData/AtmosphereAutopilot/Global_settings.cfg",
                typeof(GlobalSerializable));
        }

        public bool Deserialize(string node_name, string filename, Type attribute_type)
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
                    AutoSerialization.DeserializeFromNode(node, this, attribute_type);
                    OnDeserialize(node, attribute_type);
                    return true;
                }
            }
            return false;
        }

        public virtual void OnDeserialize(ConfigNode node, Type attribute_type) { }

        public void Serialize(string node_name, string filename, Type attribute_type)
        {
            ConfigNode fileNode = ConfigNode.Load(filename);
            if (fileNode == null)
                fileNode = new ConfigNode();
            fileNode.RemoveNode(node_name);
            ConfigNode node = new ConfigNode(node_name);
            AutoSerialization.SerializeToNode(node, this, attribute_type);
            OnSerialize(node, attribute_type);
            fileNode.AddNode(node);
            fileNode.Save(filename);
        }

        public virtual void OnSerialize(ConfigNode node, Type attribute_type) { }

        #endregion


     
        #region GUI

        string module_name;
        int wnd_id;
        protected bool gui_shown = false;
        protected Rect window = new Rect(50.0f, 80.0f, 200.0f, 150.0f);

        public bool IsDrawn()
        {
            return gui_shown;
        }

        public void OnGUI()
        {
            if (!gui_shown)
                return;
            window = GUILayout.Window(wnd_id, window, _drawGUI, module_name);
        }

        public virtual void _drawGUI(int id)
        {
            GUILayout.BeginVertical();
            AutoGUI.AutoDrawObject(this);
            GUILayout.EndVertical();
        }

        public bool ToggleGUI()
        {
            return gui_shown = !gui_shown;
        }

        #endregion
    }
}
