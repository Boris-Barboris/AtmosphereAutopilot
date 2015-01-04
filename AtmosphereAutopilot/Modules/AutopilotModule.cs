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
            return AutoSerialization.Deserialize(this, module_name.Replace(' ', '_'),
                KSPUtil.ApplicationRootPath + "GameData/AtmosphereAutopilot/" + vessel.vesselName + ".cfg",
                typeof(VesselSerializable), OnDeserialize);
        }

        public bool DeserializeGlobalSpecific()
        {
            return AutoSerialization.Deserialize(this, module_name.Replace(' ', '_'), 
                KSPUtil.ApplicationRootPath + "GameData/AtmosphereAutopilot/Global_settings.cfg",
                typeof(GlobalSerializable), OnDeserialize);
        }

        public void Serialize()
        {
            AutoSerialization.Serialize(this, module_name.Replace(' ', '_'),
                KSPUtil.ApplicationRootPath + "GameData/AtmosphereAutopilot/" + vessel.vesselName + ".cfg",
                typeof(VesselSerializable), OnSerialize);
            AutoSerialization.Serialize(this, module_name.Replace(' ', '_'),
                KSPUtil.ApplicationRootPath + "GameData/AtmosphereAutopilot/Global_settings.cfg",
                typeof(GlobalSerializable), OnSerialize);
        }

        public bool Deserialize()
        {
            return (DeserializeVesselSpecific() & DeserializeGlobalSpecific());
        }

        protected virtual void OnDeserialize(ConfigNode node, Type attribute_type) { }

        protected virtual void OnSerialize(ConfigNode node, Type attribute_type) { }

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
