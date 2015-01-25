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

		// Map of vessel - module manager relation
		Dictionary<Vessel, TopModuleManager> module_managers = new Dictionary<Vessel, TopModuleManager>();

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
            autopilot_module_types.Add(typeof(InstantControlModel));
            autopilot_module_types.Add(typeof(MediumFlightModel));
            autopilot_module_types.Add(typeof(PitchAngularAccController));
            autopilot_module_types.Add(typeof(PitchAngularVelocityController));
            autopilot_module_types.Add(typeof(RollAngularAccController));
			autopilot_module_types.Add(typeof(RollAngularVelocityController));
			autopilot_module_types.Add(typeof(YawAngularAccController));
			autopilot_module_types.Add(typeof(YawAngularVelocityController));
        }

        void initialize_hotkeys()
        {
            module_hotkeys[typeof(TopModuleManager)] = KeyCode.P;
        }

        void serialize_active_modules()
        {
			foreach (var pair in autopilot_module_lists[ActiveVessel])
            {
                ISerializable s = pair.Value as ISerializable;
                if (s != null)
                    s.Serialize();
            }
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
				autopilot_module_lists[v] = new Dictionary<Type, AutopilotModule>();
			if (!module_managers.ContainsKey(v))
				module_managers[v] = new TopModuleManager(v);			
        }

        private void vesselSwitch(Vessel v)
        {
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
            {
                IWindow s = pair.Value as IWindow;
                if (s != null)
                    s.OnGUI();
            }
        }

        public void OnHideUI()
        {
            applauncher.HideGUI();
			if (ActiveVessel == null)
				return;
			foreach (var pair in autopilot_module_lists[ActiveVessel])
            {
                IWindow s = pair.Value as IWindow;
                if (s != null)
                    s.HideGUI();
            }
        }

        public void OnShowUI()
        {
            applauncher.UnHideGUI();
			if (ActiveVessel == null)
				return;
			foreach (var pair in autopilot_module_lists[ActiveVessel])
            {
                IWindow s = pair.Value as IWindow;
                if (s != null)
                    s.UnHideGUI();
            }
        }

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
				if (pair.Key == typeof(TopModuleManager))
				{
					if (Input.GetKeyDown(pair.Value) &&
						module_managers.ContainsKey(ActiveVessel))
					{
						TopModuleManager module = module_managers[ActiveVessel];
						if (module != null)
						{
							module.Active = !module.Active;
							MessageManager.post_status_message(module.ModuleName + (module.Active ? " enabled" : " disabled"));
						}
					}
				}
			}
        }
    }
}
