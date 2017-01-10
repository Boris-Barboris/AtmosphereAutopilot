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
using UnityEngine;
using System.Reflection;

namespace AtmosphereAutopilot
{
    public sealed class TopModuleManager : StateController
    {
        internal TopModuleManager(Vessel vessel)
            : base(vessel, "Autopilot module manager", 24888888)
        {
            cur_ves_modules = AtmosphereAutopilot.Instance.autopilot_module_lists[vessel];
            settings_wnd = new CraftSettingsWindow(this);
            settings_wnd.Deserialize();
        }

        // All finished autopilot modules, created for this vessel
        Dictionary<Type, AutopilotModule> cur_ves_modules;

        // All high-level autopilot modules, created for this vessel
        Dictionary<Type, StateController> HighLevelControllers = new Dictionary<Type, StateController>();

        // Currently active high-level autopilot
        StateController active_controller = null;

        // settings window
        CraftSettingsWindow settings_wnd;

        public void create_context()
        {
            // We need to create all those modules. Module type needs to define constructor of
            // Constructor(Vessel v) prototype.
            foreach (var module_type in AtmosphereAutopilot.Instance.autopilot_module_types)
            {
                if (module_type.Equals(typeof(TopModuleManager)))
                    continue;
                var constructor = module_type.GetConstructor(BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance, null,
                    new[] { typeof(Vessel) }, null);
                if (constructor == null)
                    throw new NullReferenceException(module_type.Name + " module has no void(Vessel) constructor.");
                cur_ves_modules[module_type] = (AutopilotModule)constructor.Invoke(new[] { vessel });
            }
            // Then we need to resolve relations and deserialize
            foreach (var module_type in cur_ves_modules.Keys)
                if (!module_type.Equals(typeof(TopModuleManager)))
                {
                    cur_ves_modules[module_type].InitializeDependencies(cur_ves_modules);
                    cur_ves_modules[module_type].Deserialize();
                }

            // Move all high-level controllers to list
            foreach (var module_type in cur_ves_modules.Keys)
                if (!module_type.Equals(typeof(TopModuleManager)))
                    if (module_type.IsSubclassOf(typeof(StateController)))
                        HighLevelControllers.Add(module_type, (StateController)cur_ves_modules[module_type]);

            if (HighLevelControllers.Count <= 0)
                throw new InvalidOperationException("No high-level autopilot modules were found");
            else
                active_controller = HighLevelControllers[typeof(StandardFlyByWire)];

            // map settings window to modules
            settings_wnd.map_modues();
        }

        protected override void OnActivate()
        {
            // If this top_manager is the only module loaded for this vessel
            if (cur_ves_modules.Count == 1)
                create_context();

            vessel.ActionGroups.SetGroup(KSPActionGroup.SAS, false);
            if (active_controller != null)
                active_controller.Activate();
            vessel.OnAutopilotUpdate += new FlightInputCallback(ApplyControl);
            MessageManager.post_status_message("Autopilot module manager enabled");
        }

        protected override void OnDeactivate()
        {
            foreach (var module_type in cur_ves_modules.Keys)
                if (!module_type.Equals(typeof(TopModuleManager)))
                {
                    cur_ves_modules[module_type].Serialize();
                    cur_ves_modules[module_type].Deactivate();
                }
            vessel.OnAutopilotUpdate -= new FlightInputCallback(ApplyControl);
            MessageManager.post_status_message("Autopilot module manager disabled");
        }

        public override void ApplyControl(FlightCtrlState state)
        {
            if (active_controller != null && vessel.state != Vessel.State.DEAD)
                active_controller.ApplyControl(state);
        }

        protected override void _drawGUI(int id)
        {
            close_button();
            GUILayout.BeginVertical();
            Active = GUILayout.Toggle(Active, "MASTER SWITCH", GUIStyles.toggleButtonStyle);
            bool show_settings = GUILayout.Toggle(settings_wnd.IsShown(), "Craft settings", GUIStyles.toggleButtonStyle);
            if (show_settings)
                settings_wnd.ShowGUI();
            else
                settings_wnd.UnShowGUI();
            GUILayout.Space(10);
            foreach (var controller in HighLevelControllers.Values)
            {
                GUILayout.BeginHorizontal();
                bool pressed = GUILayout.Toggle(active_controller == controller, controller.ModuleName,
                    GUIStyles.toggleButtonStyle, GUILayout.Width(155.0f), GUILayout.ExpandWidth(false));
                if (pressed && !controller.Active)
                {
                    if (Active)
                    {
                        // we activate new module
                        bool activation = true;
                        if (active_controller != null)
                        {
                            activation = false;
                            active_controller.Deactivate();
                        }
                        controller.Activate();
                        if (!activation)
                            (cur_ves_modules[typeof(FlightModel)] as FlightModel).sequential_dt = true;
                    }
                    active_controller = controller;
                }
                bool is_shown = GUILayout.Toggle(controller.IsShown(), "GUI", GUIStyles.toggleButtonStyle);
                if (is_shown)
                    controller.ShowGUI();
                else
                    controller.UnShowGUI();
                GUILayout.EndHorizontal();
            }
            GUILayout.EndVertical();
            GUI.DragWindow();
        }

        protected override void OnGUICustom()
        {
            if (settings_wnd.mapped)
                settings_wnd.OnGUI();
            else
                if (settings_wnd.IsShown())
            {
                create_context();
                settings_wnd.OnGUI();
            }
        }

        /// <summary>
        /// Set desired controller as active.
        /// </summary>
        /// <param name="controllerType">Type of StateController to activate.</param>
        /// <returns>Instance of activated controller or null if something failed.</returns>
        public StateController activateAutopilot(Type controllerType)
        {
            if (!Active)
                Active = true;
            if (HighLevelControllers.Keys.Contains(controllerType))
            {
                bool activation = true;
                StateController cres = HighLevelControllers[controllerType];
                if (cres != active_controller)
                    if (active_controller != null)
                    {
                        active_controller.Deactivate();
                        activation = false;
                    }
                cres.Activate();
                active_controller = cres;
                if (!activation)
                    (cur_ves_modules[typeof(FlightModel)] as FlightModel).sequential_dt = true;
                return cres;
            }
            else
                return null;
        }

        [GlobalSerializable("master_switch_key")]
        [AutoHotkeyAttr("Master switch")]
        static KeyCode master_switch_key = KeyCode.P;

        public override void OnUpdate()
        {
            if (Input.GetKeyDown(master_switch_key))
                if (Input.GetKey(KeyCode.LeftShift) || Input.GetKey(KeyCode.RightShift))
                {
                    ToggleGUI();
                }
                else
                {
                    Active = !Active;
                    AtmosphereAutopilot.Instance.mainMenuGUIUpdate();
                }
        }

        #region SettingsWindow

        public class CraftSettingsWindow : GUIWindow
        {
            TopModuleManager owner;

            public CraftSettingsWindow(TopModuleManager manager)
                : base("Craft settings", 29414122, new Rect(50.0f, 80.0f, 200.0f, 50.0f))
            {
                owner = manager;
            }

            PitchAngularVelocityController pvc;
            YawAngularVelocityController yvc;
            RollAngularVelocityController rvc;

            public bool mapped = false;

            public void map_modues()
            {
                pvc = owner.cur_ves_modules[typeof(PitchAngularVelocityController)] as PitchAngularVelocityController;
                yvc = owner.cur_ves_modules[typeof(YawAngularVelocityController)] as YawAngularVelocityController;
                rvc = owner.cur_ves_modules[typeof(RollAngularVelocityController)] as RollAngularVelocityController;
                mapped = true;
            }

            protected override void _drawGUI(int id)
            {
                GUILayout.BeginVertical();

                // Moderation sections
                GUILayout.BeginHorizontal();
                pvc.moderate_aoa = GUILayout.Toggle(pvc.moderate_aoa, "Moderate AoA", GUIStyles.toggleButtonStyle);
                float.TryParse(GUILayout.TextField(pvc.max_aoa.ToString("G4"), GUIStyles.textBoxStyle), out pvc.max_aoa);
                GUILayout.EndHorizontal();
                GUILayout.BeginHorizontal();
                pvc.moderate_g = GUILayout.Toggle(pvc.moderate_g, "Moderate G", GUIStyles.toggleButtonStyle);
                float.TryParse(GUILayout.TextField(pvc.max_g_force.ToString("G4"), GUIStyles.textBoxStyle), out pvc.max_g_force);
                GUILayout.EndHorizontal();
                GUILayout.BeginHorizontal();
                yvc.moderate_aoa = GUILayout.Toggle(yvc.moderate_aoa, "Moderate Sideslip", GUIStyles.toggleButtonStyle);
                float.TryParse(GUILayout.TextField(yvc.max_aoa.ToString("G4"), GUIStyles.textBoxStyle), out yvc.max_aoa);
                GUILayout.EndHorizontal();
                GUILayout.BeginHorizontal();
                yvc.moderate_g = GUILayout.Toggle(yvc.moderate_g, "Moderate side-G", GUIStyles.toggleButtonStyle);
                float.TryParse(GUILayout.TextField(yvc.max_g_force.ToString("G4"), GUIStyles.textBoxStyle), out yvc.max_g_force);
                GUILayout.EndHorizontal();

                // rotation rate limits
                GUILayout.BeginHorizontal();
                GUILayout.Label("pitch rate limit", GUIStyles.labelStyleLeft);
                float.TryParse(GUILayout.TextField(pvc.max_v_construction.ToString("G4"), GUIStyles.textBoxStyle), out pvc.max_v_construction);
                GUILayout.EndHorizontal();
                GUILayout.BeginHorizontal();
                GUILayout.Label("roll rate limit", GUIStyles.labelStyleLeft);
                float.TryParse(GUILayout.TextField(rvc.max_v_construction.ToString("G4"), GUIStyles.textBoxStyle), out rvc.max_v_construction);
                GUILayout.EndHorizontal();
                GUILayout.BeginHorizontal();
                GUILayout.Label("yaw rate limit", GUIStyles.labelStyleLeft);
                float.TryParse(GUILayout.TextField(yvc.max_v_construction.ToString("G4"), GUIStyles.textBoxStyle), out yvc.max_v_construction);
                GUILayout.EndHorizontal();

                // wing leveler
                rvc.wing_leveler = GUILayout.Toggle(rvc.wing_leveler, "Snap wings to level", GUIStyles.toggleButtonStyle);

                // profile section

                // profile creation section
                GUILayout.Space(8.0f);
                GUILayout.Label("profiles", GUIStyles.labelStyleCenter);
                GUILayout.BeginHorizontal();
                profile_name = GUILayout.TextField(profile_name, GUIStyles.textBoxStyle);
                bool pressed = GUILayout.Button("save", GUIStyles.toggleButtonStyleRight, GUILayout.Width(40.0f));
                if (pressed && profile_name.Length > 0)
                {
                    // remove old profile with same name if needed
                    if (profiles.Count > 0 && profile_name != null && profile_name.Length > 0)
                    {
                        SettingsProfile old_profile = null;
                        for (int i = 0; i < profiles.Count; i++)
                        {
                            var p = profiles[i];
                            if (p.profile_name.Equals(profile_name))
                            {
                                old_profile = p;
                                break;
                            }
                        }
                        if (old_profile != null)
                            profiles.Remove(old_profile);
                    }
                    // save new profile
                    SettingsProfile profile = new SettingsProfile(this);
                    profile.profile_name = profile_name;
                    profiles.Add(profile);
                    // serialize it
                    Serialize();
                }
                GUILayout.EndHorizontal();
                // profile selection section
                for (int i = 0; i < profiles.Count; i++)
                {
                    GUILayout.BeginHorizontal();
                    var s = profiles[i];
                    pressed = GUILayout.Button(s.profile_name, GUIStyles.toggleButtonStyle);
                    if (pressed)
                    {
                        // let's actiavte this profile
                        s.Apply(this);
                        profile_name = s.profile_name;
                    }
                    pressed = GUILayout.Button("del", GUIStyles.toggleButtonStyleRight, GUILayout.Width(30.0f));
                    if (pressed)
                    {
                        // let's delete this profile
                        profiles.Remove(s);
                        window.height = 0.0f;       // compact window
                        Serialize();
                    }
                    GUILayout.EndHorizontal();
                }

                GUILayout.EndVertical();
                GUI.DragWindow();
            }

            [GlobalSerializable("window_x")]
            protected float WindowLeft { get { return window.xMin; } set { window.xMin = value; } }

            [GlobalSerializable("window_y")]
            protected float WindowTop { get { return window.yMin; } set { window.yMin = value; } }

            [GlobalSerializable("window_width")]
            protected float WindowWidth { get { return window.width; } set { window.width = value; } }

            string profile_name = "";

            public class SettingsProfile
            {
                public string profile_name;
                public bool moderate_aoa, moderate_sideslip, moderate_g, moderate_g_hor, wing_leveler;
                public float max_aoa, max_sideslip, max_g, max_g_hor;
                public float ptich_v, roll_v, yaw_v;

                public void Apply(CraftSettingsWindow wnd)
                {
                    wnd.pvc.moderate_aoa = moderate_aoa;
                    wnd.pvc.max_aoa = max_aoa;
                    wnd.pvc.moderate_g = moderate_g;
                    wnd.pvc.max_g_force = max_g;
                    wnd.pvc.max_v_construction = ptich_v;

                    wnd.yvc.moderate_aoa = moderate_sideslip;
                    wnd.yvc.max_aoa = max_sideslip;
                    wnd.yvc.moderate_g = moderate_g_hor;
                    wnd.yvc.max_g_force = max_g_hor;
                    wnd.yvc.max_v_construction = yaw_v;

                    wnd.rvc.wing_leveler = wing_leveler;
                    wnd.rvc.max_v_construction = roll_v;
                }

                public SettingsProfile(CraftSettingsWindow wnd)
                {
                    profile_name = wnd.profile_name;

                    moderate_aoa = wnd.pvc.moderate_aoa;
                    max_aoa = wnd.pvc.max_aoa;
                    moderate_g = wnd.pvc.moderate_g;
                    max_g = wnd.pvc.max_g_force; ;
                    ptich_v = wnd.pvc.max_v_construction;

                    moderate_sideslip = wnd.yvc.moderate_aoa;
                    max_sideslip = wnd.yvc.max_aoa;
                    moderate_g_hor = wnd.yvc.moderate_g;
                    max_g_hor = wnd.yvc.max_g_force;
                    yaw_v = wnd.yvc.max_v_construction;

                    wing_leveler = wnd.rvc.wing_leveler;
                    roll_v = wnd.rvc.max_v_construction;
                }

                public SettingsProfile() { }

                public void Serialize(ConfigNode node)
                {
                    ConfigNode profileNode = new ConfigNode(profile_name);
                    profileNode.AddValue("moderate_aoa", moderate_aoa);
                    profileNode.AddValue("moderate_sideslip", moderate_sideslip);
                    profileNode.AddValue("moderate_g", moderate_g);
                    profileNode.AddValue("moderate_g_hor", moderate_g_hor);
                    profileNode.AddValue("wing_leveler", wing_leveler);
                    profileNode.AddValue("max_aoa", max_aoa);
                    profileNode.AddValue("max_g", max_g);
                    profileNode.AddValue("max_sideslip", max_sideslip);
                    profileNode.AddValue("max_g_hor", max_g_hor);
                    profileNode.AddValue("ptich_v", ptich_v);
                    profileNode.AddValue("roll_v", roll_v);
                    profileNode.AddValue("yaw_v", yaw_v);
                    node.AddNode(profileNode);
                }

                public static SettingsProfile Deserialize(ConfigNode node)
                {
                    SettingsProfile prof = new SettingsProfile();
                    prof.profile_name = node.name;
                    bool.TryParse(node.GetValue("moderate_aoa"), out prof.moderate_aoa);
                    bool.TryParse(node.GetValue("moderate_sideslip"), out prof.moderate_sideslip);
                    bool.TryParse(node.GetValue("moderate_g"), out prof.moderate_g);
                    bool.TryParse(node.GetValue("moderate_g_hor"), out prof.moderate_g_hor);
                    bool.TryParse(node.GetValue("wing_leveler"), out prof.wing_leveler);
                    float.TryParse(node.GetValue("max_aoa"), out prof.max_aoa);
                    float.TryParse(node.GetValue("max_g"), out prof.max_g);
                    float.TryParse(node.GetValue("max_sideslip"), out prof.max_sideslip);
                    float.TryParse(node.GetValue("max_g_hor"), out prof.max_g_hor);
                    float.TryParse(node.GetValue("ptich_v"), out prof.ptich_v);
                    float.TryParse(node.GetValue("roll_v"), out prof.roll_v);
                    float.TryParse(node.GetValue("yaw_v"), out prof.yaw_v);
                    return prof;
                }
            }

            static List<SettingsProfile> profiles = new List<SettingsProfile>();

            public void Serialize()
            {
                AutoSerialization.Serialize(this, "settings_wnd", KSPUtil.ApplicationRootPath + "GameData/AtmosphereAutopilot/Global_settings.txt",
                    typeof(GlobalSerializable), OnSerialize);
            }

            void OnSerialize(ConfigNode node, Type attType)
            {
                ConfigNode profiles_list_node = new ConfigNode("profiles");
                foreach (var p in profiles)
                    p.Serialize(profiles_list_node);
                node.AddNode(profiles_list_node);
            }

            public bool Deserialize()
            {
                return AutoSerialization.Deserialize(this, "settings_wnd", KSPUtil.ApplicationRootPath + "GameData/AtmosphereAutopilot/Global_settings.txt",
                    typeof(GlobalSerializable), OnDeserialize);
            }

            void OnDeserialize(ConfigNode node, Type attType)
            {
                ConfigNode profiles_list_node = node.GetNode("profiles");
                if (profiles_list_node == null)
                    return;
                profiles.Clear();
                for (int i = 0; i < profiles_list_node.nodes.Count; i++)
                {
                    var pf = profiles_list_node.nodes[i];
                    profiles.Add(SettingsProfile.Deserialize(pf));
                }
            }
        }

        #endregion SettingsWindow

        public override void Serialize()
        {
            base.Serialize();
            settings_wnd.Serialize();
        }

        public override bool Deserialize()
        {
            return base.Deserialize() && settings_wnd.Deserialize();
        }

    }
}
