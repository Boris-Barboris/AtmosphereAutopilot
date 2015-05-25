using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;

namespace AtmosphereAutopilot
{
    [KSPAddon(KSPAddon.Startup.Instantly, true)]
    public class AtmosphereAutopilot: MonoBehaviour
    {
		// List of all autopilot modules, that need to be created for vessel
        internal List<Type> autopilot_module_types = new List<Type>();

		// Map of vessel - module list relation
        internal Dictionary<Vessel, Dictionary<Type, AutopilotModule>> autopilot_module_lists =
			new Dictionary<Vessel, Dictionary<Type, AutopilotModule>>();

		// Hotkeys for module activation
        Dictionary<Type, KeyCode> module_hotkeys = new Dictionary<Type, KeyCode>();

		AppLauncherWindow applauncher = new AppLauncherWindow();

        public static AtmosphereAutopilot Instance { get; private set; }

		public Vessel ActiveVessel { get; private set; }

        public void Start()
        {
            Debug.Log("[Autopilot]: AtmosphereAutopilot starting up!"); 
            DontDestroyOnLoad(this);
            initialize_types();
            initialize_hotkeys();
            GameEvents.onVesselChange.Add(vesselSwitch);
            GameEvents.onGameSceneLoadRequested.Add(sceneSwitch);
            GameEvents.onHideUI.Add(OnHideUI);
            GameEvents.onShowUI.Add(OnShowUI);
            GameEvents.onGUIApplicationLauncherReady.Add(onAppLauncherLoad);
            GameEvents.onGUIApplicationLauncherDestroyed.Add(onAppLauncherDestroy);
            Instance = this;
			ActiveVessel = null;
        }

        void initialize_types()
        {
            var lListOfBs = (from lAssembly in AppDomain.CurrentDomain.GetAssemblies()
                             from lType in lAssembly.GetTypes()
                             where lType.IsSubclassOf(typeof(AutopilotModule))
                             where lType.IsSealed
                             select lType);

            autopilot_module_types.AddRange(lListOfBs);
        }

        void initialize_hotkeys()
        {
            module_hotkeys[typeof(TopModuleManager)] = KeyCode.P;
        }

        void serialize_active_modules()
        {
            if (ActiveVessel == null)
                return;
            foreach (var module in autopilot_module_lists[ActiveVessel].Values)
                module.Serialize();
        }

        public void OnDestroy()
        {
            sceneSwitch(GameScenes.CREDITS);
        }

        private void sceneSwitch(GameScenes scenes)
        {
            serialize_active_modules();
			if (scenes != GameScenes.FLIGHT)
				ActiveVessel = null;
        }

		private void load_manager_for_vessel(Vessel v)
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

        private void vesselSwitch(Vessel v)
        {
            serialize_active_modules();
            Debug.Log("[Autopilot] vessel switch");
            load_manager_for_vessel(v);
			ActiveVessel = v;
        }

		public Dictionary<Type, AutopilotModule> getCurVesselModules()
		{
			return autopilot_module_lists[ActiveVessel];
		}


		#region AppLauncherSection

		ApplicationLauncherButton launcher_btn;

        void onAppLauncherLoad()
        {
            GameEvents.onGUIApplicationLauncherReady.Remove(onAppLauncherLoad);
            launcher_btn = ApplicationLauncher.Instance.AddModApplication(
				OnALTrue, OnALFalse, OnHover, OnALUnHover, null, null, ApplicationLauncher.AppScenes.FLIGHT,
                GameDatabase.Instance.GetTexture("AtmosphereAutopilot/icon", false));
        }

        void onAppLauncherDestroy()
        {
            GameEvents.onGUIApplicationLauncherReady.Add(onAppLauncherLoad);
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

        public void OnGUI()
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

        public void OnHideUI()
        {
            applauncher.HideGUI();
			if (ActiveVessel == null)
				return;
            foreach (var pair in autopilot_module_lists[ActiveVessel])
                pair.Value.HideGUI();
        }

        public void OnShowUI()
        {
            applauncher.UnHideGUI();
			if (ActiveVessel == null)
				return;
            foreach (var pair in autopilot_module_lists[ActiveVessel])
                pair.Value.UnHideGUI();
        }

		#endregion

		public void Update()
        {
            if (InputLockManager.IsLocked(ControlTypes.ALL_SHIP_CONTROLS))
                return;
            if (!HighLogic.LoadedSceneIsFlight)
                return;
			if (ActiveVessel == null)
				return;

            bool mod = GameSettings.MODIFIER_KEY.GetKey();

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
