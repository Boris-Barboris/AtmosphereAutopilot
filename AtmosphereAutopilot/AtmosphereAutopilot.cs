/*
Copyright 2015, Boris-Barboris

This file is part of Atmosphere Autopilot.
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
    [KSPAddon(KSPAddon.Startup.MainMenu, true)]
    public sealed class AtmosphereAutopilot: MonoBehaviour
    {
		// List of all autopilot modules, that need to be created for vessel
        internal List<Type> autopilot_module_types = new List<Type>();

		// Map of vessel - module list relation
        internal Dictionary<Vessel, Dictionary<Type, AutopilotModule>> autopilot_module_lists =
			new Dictionary<Vessel, Dictionary<Type, AutopilotModule>>();

		// Hotkeys for module activation
        Dictionary<Type, KeyCode> module_hotkeys = new Dictionary<Type, KeyCode>();

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
            Debug.Log("[Autopilot]: AtmosphereAutopilot starting up!"); 
            DontDestroyOnLoad(this);
            classify_aero();
            initialize_types();
            initialize_hotkeys();
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

        public static AerodinamycsModel AeroModel = AerodinamycsModel.Stock;

        void classify_aero()
        {
            foreach (var a in AppDomain.CurrentDomain.GetAssemblies())
            {
                if (a.FullName.Equals("FerramAerospaceResearch"))
                {
                    AeroModel = AerodinamycsModel.FAR;
                    return;
                }
            }
        }

        void initialize_types()
        {
            // Find all sealed children of AutopilotModule and treat them as complete autopilot modules
            var lListOfBs = (from lAssembly in AppDomain.CurrentDomain.GetAssemblies()
                             from lType in lAssembly.GetTypes()
                             where lType.IsSubclassOf(typeof(AutopilotModule))
                             where lType.IsSealed
                             select lType);
            autopilot_module_types.AddRange(lListOfBs);
        }

        void initialize_hotkeys()
        {
            module_hotkeys[typeof(TopModuleManager)] = KeyCode.P;       // press P to activate master switch
        }

        // Thread for computational-heavy tasks
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
                Debug.Log("[Autopilot] new vessel, creating new module map");
                autopilot_module_lists[v] = new Dictionary<Type, AutopilotModule>();
            }
            if (!autopilot_module_lists[v].ContainsKey(typeof(TopModuleManager)))
                autopilot_module_lists[v][typeof(TopModuleManager)] = new TopModuleManager(v);		
        }

        void vesselSwitch(Vessel v)
        {
            serialize_active_modules();
            Debug.Log("[Autopilot] vessel switch");
            load_manager_for_vessel(v);
			ActiveVessel = v;
        }

        void clean_modules()
        {
            var vesselsToRemove = autopilot_module_lists.Keys.Where(v => v.state == Vessel.State.DEAD).ToArray();
            foreach (var v in vesselsToRemove)
            {
                var manager = autopilot_module_lists[v][typeof(TopModuleManager)];
                manager.Deactivate();
                autopilot_module_lists.Remove(v);
            }
        }

        /// <summary>
        /// Get set of AutopilotModule instances, created for arbitrary vessel.
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
						OnALTrue, OnALFalse, OnHover, OnALUnHover, null, null, ApplicationLauncher.AppScenes.FLIGHT,
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
			if (ActiveVessel == null)
				return;
            if (!styles_init)
            {
                GUIStyles.Init();
                styles_init = true;
            }
            GUI.skin = GUIStyles.skin;
            applauncher.OnGUI();
            foreach (var pair in autopilot_module_lists[ActiveVessel])
                pair.Value.OnGUI();
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
            // Handle keyboard hotkeys here
            if (InputLockManager.IsLocked(ControlTypes.ALL_SHIP_CONTROLS))
                return;
            if (!HighLogic.LoadedSceneIsFlight)
                return;
			if (ActiveVessel == null)
				return;
			foreach (var pair in module_hotkeys)
			{
				if (Input.GetKeyDown(pair.Value) &&
                    autopilot_module_lists.ContainsKey(ActiveVessel))
				{
                    AutopilotModule module = autopilot_module_lists[ActiveVessel][pair.Key];
					module.Active = !module.Active;
					MessageManager.post_status_message(module.ModuleName + (module.Active ? " enabled" : " disabled"));
				}
			}
        }
    }
}
