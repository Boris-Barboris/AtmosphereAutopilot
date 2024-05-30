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
using System.ComponentModel;
using System.Linq;
using System.Text;
using UnityEngine;

namespace AtmosphereAutopilot
{

    /// <summary>
    /// Represents autopilot module, wich can be turned on and off.
    /// It has GUI and can be serialized. Derived classes need to implement
    /// OnActivate and OnDeactivate.
    /// </summary>
    public abstract class AutopilotModule : GUIWindow, ISerializable
    {
        public const int PITCH = 0;
        public const int ROLL = 1;
        public const int YAW = 2;

        public const float rad2dgr = (float)(180.0 / Math.PI);
        public const float dgr2rad = (float)(Math.PI / 180.0);

        protected Vessel vessel = null;
        protected bool enabled = false;
        protected string module_name, module_compact_name;

        protected static readonly Dictionary<string, string> compact_name_map = new Dictionary<string, string> {
            //{ "Mouse Director", "MD" }, //I'm a keyboard pleb, so I exclude this from the compact UI by commenting it out here
            { "Cruise Flight controller", "CF" },
            { "AoA-hold controller", "AoA" },
            { "Standard Fly-By-Wire", "FBW" },
        };

        protected AutopilotModule(Vessel v, int wnd_id, string module_name):
            base(module_name, wnd_id, new Rect(50.0f, 80.0f, 200.0f, 50.0f))
        {
            vessel = v;
            this.module_name = module_name;
            try {
                this.module_compact_name = compact_name_map[this.module_name];
            } catch (KeyNotFoundException) {
                this.module_compact_name = "";
            }
        }

        /// <summary>
        /// If this module is dependent on other modules, you should get references to
        /// required ones in this call.
        /// </summary>
        /// <param name="modules">Map of autopilot modules, instanced for this vessel.</param>
        public virtual void InitializeDependencies(Dictionary<Type, AutopilotModule> modules) { }

        public void Activate()
        {
            if (!enabled)
                OnActivate();
            enabled = true;
        }
        
        public string ModuleName { get { return module_name; } }
        public string ModuleCompactName { get { return module_compact_name; } }

        protected abstract void OnActivate();

        public void Deactivate()
        {
            if (enabled)
                OnDeactivate();
            enabled = false;
        }

        protected abstract void OnDeactivate();

        public bool Active
        {
            get { return enabled; }
            set
            {
                if (value)
                    if (!enabled)
                        Activate();
                    else {}
                else
                    if (enabled)
                        Deactivate();
            }
        }

        /// <summary>
        /// Is called on Update Unity event for current vessel every frame
        /// </summary>
        public virtual void OnUpdate() { }

        #region Serialization

        /// <summary>
        /// Deserialize vessel-specific fields. Optional OnDeserialize callback is used.
        /// </summary>
        public bool DeserializeVesselSpecific()
        {
            return AutoSerialization.Deserialize(this, module_name.Replace(' ', '_'),
                KSPUtil.ApplicationRootPath + "GameData/AtmosphereAutopilot/designs/" + KSPUtil.SanitizeFilename(vessel.vesselName) + ".txt",
                typeof(VesselSerializable), OnDeserialize);
        }

        /// <summary>
        /// Deserialize global fields. Optional OnDeserialize callback is used.
        /// </summary>
        public bool DeserializeGlobalSpecific()
        {
            return AutoSerialization.Deserialize(this, module_name.Replace(' ', '_'),
                KSPUtil.ApplicationRootPath + "GameData/AtmosphereAutopilot/Global_settings.txt",
                typeof(GlobalSerializable), OnDeserialize);
        }

        /// <summary>
        /// Serialize global and vessel data to files. BeforeSerialized and OnSerialize callbacks are used.
        /// </summary>
        public virtual void Serialize()
        {
            BeforeSerialized();
            AutoSerialization.Serialize(this, module_name.Replace(' ', '_'),
                KSPUtil.ApplicationRootPath + "GameData/AtmosphereAutopilot/designs/" + KSPUtil.SanitizeFilename(vessel.vesselName) + ".txt",
                typeof(VesselSerializable), OnSerialize);
            AutoSerialization.Serialize(this, module_name.Replace(' ', '_'),
                KSPUtil.ApplicationRootPath + "GameData/AtmosphereAutopilot/Global_settings.txt",
                typeof(GlobalSerializable), OnSerialize);
        }

        protected virtual void BeforeSerialized() { }

        protected virtual void BeforeDeserialized() { }

        /// <summary>
        /// Deserialize global data and then vessel-specific. Optional BeforeDeserialized and OnDeserialize 
        /// callbacks are used.
        /// </summary>
        /// <returns>true if nothing crashed</returns>
        public virtual bool Deserialize()
        {
            BeforeDeserialized();
            return (DeserializeGlobalSpecific() & DeserializeVesselSpecific());
        }

        protected virtual void OnDeserialize(ConfigNode node, Type attribute_type) { }

        protected virtual void OnSerialize(ConfigNode node, Type attribute_type) { }

        #endregion
     
        #region GUI

        [GlobalSerializable("window_x")]
        protected float WindowLeft { get { return windowPosition.xMin; } 
            set 
            {
                float width = windowPosition.width;
                windowPosition.xMin = value;
                windowPosition.xMax = windowPosition.xMin + width;
            } 
        }

        [GlobalSerializable("window_y")]
        protected float WindowTop { get { return windowPosition.yMin; } 
            set 
            {
                float height = windowPosition.height;
                windowPosition.yMin = value;
                windowPosition.yMax = windowPosition.yMin + height;
            } 
        }

        //[GlobalSerializable("window_width")]
        protected float WindowWidth { get { return windowPosition.width; } set { windowPosition.width = value; } }

        /// <inheritdoc />
        protected override void _drawGUI(int id)
        {
            GUILayout.BeginVertical();
            AutoGUI.AutoDrawObject(this);
            GUILayout.EndVertical();
        }

        #endregion
    }
}
