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
using KSP.UI.Screens;
//using ToolbarWrapper;

namespace AtmosphereAutopilot
{
    [KSPAddon(KSPAddon.Startup.MainMenu, true)]
    public sealed class AtmosphereAutopilot: MonoBehaviour
    {
        // List of all autopilot modules, that need to be created for vessel
        internal List<Type> autopilot_module_types = new List<Type>();

        // Map of vessel - module list relation
        internal Dictionary<Vessel, Dictionary<Type, AutopilotModule>> autopilot_module_lists =
            new Dictionary<Vessel, Dictionary<Type, AutopilotModule>>(new VesselOldComparator());

        // Application launcher window
        AppLauncherWindow applauncher = new AppLauncherWindow();

        /// <summary>
        /// Get AtmosphereAutopilot addon class instance
        /// </summary>
        public static AtmosphereAutopilot Instance { get; private set; }

        /// <summary>
        /// Get current active (controlled by player) vessel
        /// </summary>
        public Vessel ActiveVessel { get; private set; }

        void Start()
        {
            Debug.Log("[AtmosphereAutopilot]: starting up!"); 
            DontDestroyOnLoad(this);
            determine_aerodynamics();
            get_csurf_module();
            get_gimbal_modules();
            initialize_types();
            initialize_thread();
            GameEvents.onVesselChange.Add(vesselSwitch);
            GameEvents.onGameSceneLoadRequested.Add(sceneSwitch);
            GameEvents.onHideUI.Add(OnHideUI);
            GameEvents.onShowUI.Add(OnShowUI);
            GameEvents.onGUIApplicationLauncherReady.Add(onAppLauncherLoad);
            GameEvents.onGamePause.Add(OnApplicationPause);
            GameEvents.onGameUnpause.Add(OnApplicationUnpause);
            Instance = this;
            ActiveVessel = null;
        }

        public enum AerodinamycsModel
        {
            Stock,
            FAR
        }

        /// <summary>
        /// Current aerodynamics model.
        /// </summary>
        public static AerodinamycsModel AeroModel { get; private set; }
        Assembly far_assembly;

        void determine_aerodynamics()
        {
            AeroModel = AerodinamycsModel.Stock;
            foreach (var a in AppDomain.CurrentDomain.GetAssemblies())
            {
                if (a.GetName().Name.Equals("FerramAerospaceResearch"))
                {
                    far_assembly = a;
                    AeroModel = AerodinamycsModel.FAR;
                    Debug.Log("[AtmosphereAutopilot]: FAR aerodynamics detected");
                    return;
                }
            }
        }

        /// <summary>
        /// Type of module of control surface.
        /// </summary>
        public static Type control_surface_module_type { get; private set; }

        void get_csurf_module()
        {
            if (AeroModel == AerodinamycsModel.Stock)
                control_surface_module_type = typeof(SyncModuleControlSurface);
            else
            {
                control_surface_module_type = far_assembly.GetTypes().First(t => t.Name.Equals("FARControllableSurface"));
                if (control_surface_module_type == null)
                    throw new Exception("AtmosphereAutopilot could not bind to FAR FARControllableSurface class");
            }
        }

        public static Dictionary<Type, ConstructorInfo> gimbal_module_wrapper_map = new Dictionary<Type, ConstructorInfo>(4);

        void get_gimbal_modules()
        {
            gimbal_module_wrapper_map.Add(typeof(ModuleGimbal), typeof(StockGimbal).GetConstructors()[0]);
            if (kmGimbal.do_reflections())
                gimbal_module_wrapper_map.Add(kmGimbal.gimbal_type, typeof(kmGimbal).GetConstructors()[0]);
        }

        void initialize_types()
        {
            // Find all sealed children of AutopilotModule and treat them as complete autopilot modules
            foreach (var asmbly in AppDomain.CurrentDomain.GetAssemblies())
            {
                try
                {
                    var lListOfBs = (from lType in asmbly.GetTypes()
                                     where lType.IsSubclassOf(typeof(AutopilotModule))
                                     where lType.IsSealed
                                     select lType);
                    autopilot_module_types.AddRange(lListOfBs);
                }
                catch (Exception)
                {
                    Debug.Log("[AtmosphereAutopilot]: reflection crash on " + asmbly.FullName + " assembly.");
                }
            }
        }

        // Thread for computation-heavy tasks
        BackgroundThread thread;

        /// <summary>
        /// Get background thread, used by autopilot framework
        /// </summary>
        public BackgroundThread BackgroundThread { get { return thread; } }
        
        void initialize_thread()
        {
            thread = new BackgroundThread(KSPUtil.ApplicationRootPath + "GameData/AtmosphereAutopilot");
        }

        void OnApplicationPause()
        {
            thread.Pause();
        }

        void OnApplicationUnpause()
        {
            thread.Resume();
        }

        void serialize_active_modules()
        {
            if (ActiveVessel == null)
                return;
            foreach (var module in autopilot_module_lists[ActiveVessel].Values)
                module.Serialize();
        }

        void OnDestroy()
        {
            serialize_active_modules();
            AtmosphereAutopilot.Instance.BackgroundThread.Stop();
        }

        void sceneSwitch(GameScenes scenes)
        {
            serialize_active_modules();
            clean_modules();
            if (scenes != GameScenes.FLIGHT)
            {
                ActiveVessel = null;
                AtmosphereAutopilot.Instance.BackgroundThread.Pause();
            }
            else
                AtmosphereAutopilot.Instance.BackgroundThread.Resume();
        }

        void load_manager_for_vessel(Vessel v)
        {
            if (v == null)
                return;
            if (!autopilot_module_lists.ContainsKey(v))
            {
                Debug.Log("[AtmosphereAutopilot]: new vessel, creating new module map for " + v.vesselName);
                autopilot_module_lists[v] = new Dictionary<Type, AutopilotModule>();
            }
            if (!autopilot_module_lists[v].ContainsKey(typeof(TopModuleManager)))
                autopilot_module_lists[v][typeof(TopModuleManager)] = new TopModuleManager(v);
			autopilot_module_lists[v][typeof(TopModuleManager)].Deserialize();
        }

        void vesselSwitch(Vessel v)
        {
            serialize_active_modules();
            Debug.Log("[AtmosphereAutopilot]: vessel switch to " + v.vesselName);
            load_manager_for_vessel(v);
            ActiveVessel = v;
            //Debug.Log("[AtmosphereAutopilot]: test mark1");
            // custom behaviour for FlightModel
                //for (int i = 0; i < keys.Count; i++)
                //{
                //    Debug.Log("[AtmosphereAutopilot]: hash key of vessel " + keys[i].vesselName + " = " + keys[i].GetHashCode().ToString());
                //    Debug.Log("[AtmosphereAutopilot]: contains check - " + (autopilot_module_lists.ContainsKey(keys[i]).ToString()) + " " + keys[i].vesselName);
                //}
                foreach (Vessel c in autopilot_module_lists.Keys)
                {
                    //Debug.Log("[AtmosphereAutopilot]: iter on " + ves.vesselName);
                    //if (dil == null)
                    //    Debug.Log("[AtmosphereAutopilot]: dil = null");
                    //Debug.Log("[AtmosphereAutopilot]: dil.Keys.Count = " + dil.Keys.Count.ToString());
                    if (autopilot_module_lists[c].ContainsKey(typeof(FlightModel)))
                        (autopilot_module_lists[c][typeof(FlightModel)] as FlightModel).sequential_dt = false;
                }
            //Debug.Log("[AtmosphereAutopilot]: test mark2");
        }

        void clean_modules()
        {
            Debug.Log("[AtmosphereAutopilot]: cleaning modules hash table");
            var vesselsToRemove = autopilot_module_lists.Keys.Where(v => v.state == Vessel.State.DEAD).ToArray();
            foreach (var v in vesselsToRemove)
            {
                //if (autopilot_module_lists.ContainsKey(v))
                //{
                    var manager = autopilot_module_lists[v][typeof(TopModuleManager)];
                    manager.Deactivate();
                //}
                autopilot_module_lists.Remove(v);
                Debug.Log("[AtmosphereAutopilot]: removed vessel " + v.vesselName);
                if (autopilot_module_lists.ContainsKey(v))
                    Debug.Log("[AtmosphereAutopilot]: Logical error, not removed from keys");
            }
        }

        /// <summary>
        /// Get the map of AutopilotModule instances, created for arbitrary vessel.
        /// </summary>
        public Dictionary<Type, AutopilotModule> getVesselModules(Vessel v)
        {
            if (autopilot_module_lists.ContainsKey(v))
                return autopilot_module_lists[v];
            else
                return null;
        }


        #region AppLauncherSection

        ApplicationLauncherButton launcher_btn;

        // Called when applauncher is ready for population
        void onAppLauncherLoad()
        {
            if (ApplicationLauncher.Ready)
            {
                bool hidden;
                bool contains = ApplicationLauncher.Instance.Contains(launcher_btn, out hidden);
                if (!contains)
                    launcher_btn = ApplicationLauncher.Instance.AddModApplication(
                        OnALTrue, OnALFalse, OnHover, OnALUnHover, null, null, 
                        ApplicationLauncher.AppScenes.FLIGHT | ApplicationLauncher.AppScenes.MAPVIEW,
                        GameDatabase.Instance.GetTexture("AtmosphereAutopilot/icon", false));
            }
        }

        void OnALTrue()
        {
            applauncher.ShowGUI();
            applauncher.show_while_hover = false;
        }

        void OnHover()
        {
            applauncher.ShowGUI();
            applauncher.show_while_hover = false;
            applauncher.set_y_position(Mouse.screenPos.y);
        }

        void OnALFalse()
        {
            applauncher.UnShowGUI();
        }

        void OnALUnHover()
        {
            if (!launcher_btn.toggleButton.Value)
                applauncher.show_while_hover = true;
        }

        #endregion

        #region UI

        bool styles_init = false;

        void OnGUI()
        {
            if (!styles_init)
            {
                GUIStyles.Init();
                styles_init = true;
            }
            if (ActiveVessel == null)
                return;
            if (!HighLogic.LoadedSceneIsFlight)
                return;            
            GUIStyles.set_colors();
            applauncher.OnGUI();
            foreach (var pair in autopilot_module_lists[ActiveVessel])
                pair.Value.OnGUI();
            GUIStyles.reset_colors();
        }

        void OnHideUI()
        {
            applauncher.HideGUI();
            if (ActiveVessel == null)
                return;
            foreach (var pair in autopilot_module_lists[ActiveVessel])
                pair.Value.HideGUI();
        }

        void OnShowUI()
        {
            applauncher.UnHideGUI();
            if (ActiveVessel == null)
                return;
            foreach (var pair in autopilot_module_lists[ActiveVessel])
                pair.Value.UnHideGUI();
        }

        #endregion

        void Update()
        {
            if (!HighLogic.LoadedSceneIsFlight)
                return;
            if (ActiveVessel == null)
                return;
            if (autopilot_module_lists.ContainsKey(ActiveVessel))
            {
                var module_list = autopilot_module_lists[ActiveVessel].Values.ToList();
                foreach (var module in module_list)
                    if (module.Active || module is TopModuleManager)
                        module.OnUpdate();
            }
        }
    }
}
