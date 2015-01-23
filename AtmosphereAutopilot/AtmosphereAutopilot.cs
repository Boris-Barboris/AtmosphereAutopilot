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
        [AttributeUsage(AttributeTargets.Method)]
        class ModuleConstructor : Attribute
        {
            public Type type;
            public ModuleConstructor(Type t) { type = t; }
        }

        List<Type> autopilot_module_types = new List<Type>();
        Dictionary<Type, Dictionary<Vessel, object>> autopilot_module_lists = new Dictionary<Type, Dictionary<Vessel, object>>();
        Dictionary<Type, object> cur_ves_modules = new Dictionary<Type, object>();
        Dictionary<Type, KeyCode> module_hotkeys = new Dictionary<Type, KeyCode>(); 
        
        AppLauncherWindow applauncher;

        public static AtmosphereAutopilot Instance { get; private set; }

        public void Start()
        {
            Debug.Log("[Autopilot]: AtmosphereAutopilot starting up!"); 
            DontDestroyOnLoad(this);
            initialize_types();
            initialize_module_lists();
            initialize_hotkeys();
            applauncher = new AppLauncherWindow(cur_ves_modules);
            GameEvents.onVesselChange.Add(vesselSwitch);
            GameEvents.onGameSceneLoadRequested.Add(sceneSwitch);
            GameEvents.onHideUI.Add(OnHideUI);
            GameEvents.onShowUI.Add(OnShowUI);
            GameEvents.onGUIApplicationLauncherReady.Add(onAppLauncherLoad);
            Instance = this;
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
            autopilot_module_types.Add(typeof(TopModuleManager));
        }

        void initialize_hotkeys()
        {
            module_hotkeys[typeof(PitchAngularAccController)] = KeyCode.P;
            module_hotkeys[typeof(PitchAngularVelocityController)] = KeyCode.P;
			module_hotkeys[typeof(RollAngularAccController)] = KeyCode.P;
			module_hotkeys[typeof(RollAngularVelocityController)] = KeyCode.P;
			module_hotkeys[typeof(YawAngularAccController)] = KeyCode.P;
			module_hotkeys[typeof(YawAngularVelocityController)] = KeyCode.P;
            module_hotkeys[typeof(TopModuleManager)] = KeyCode.P;
        }

        void initialize_module_lists()
        {
            foreach (Type type in autopilot_module_types)
                autopilot_module_lists[type] = new Dictionary<Vessel, object>();
        }

        void serialize_active_modules()
        {
            foreach (var pair in cur_ves_modules)
            {
                IAutoSerializable s = pair.Value as IAutoSerializable;
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
            cur_ves_modules.Clear();
        }

        private void load_module(Vessel vessel, Type type, Func<Vessel, object> constructor)
        {
            Debug.Log("[Autopilot] load_module");
            object module;
            if (!autopilot_module_lists.ContainsKey(type))
                throw new ArgumentException("[AtmosphereAutopilot]: module type " + type.Name + " is not present in autopilot_module_lists");
            if (autopilot_module_lists[type].ContainsKey(vessel))
            {
                module = autopilot_module_lists[type][vessel];
                Debug.Log("[Autopilot]: " + type.Name + " for vessel " + vessel.name + " loaded");
            }
            else
            {
                module = constructor(vessel);
                Debug.Log("[Autopilot]: " + type.Name + " for vessel " + vessel.name + " created");
                autopilot_module_lists[type][vessel] = module;
            }
            IAutoSerializable s = module as IAutoSerializable;
            if (s != null) 
                s.Deserialize();
            cur_ves_modules[type] = module;
        }

        private void load_all_modules_for_vessel(Vessel v)
        {
            Dictionary<Type, System.Reflection.MethodInfo> constructorMap = new Dictionary<Type, System.Reflection.MethodInfo>();
            var constructors = typeof(AtmosphereAutopilot).
                GetMethods(System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance).
                        Where(m => m.GetCustomAttributes(typeof(ModuleConstructor), false).Length > 0).ToArray();
            foreach (var cons in constructors)
            {
                var att = cons.GetCustomAttributes(typeof(ModuleConstructor), false).First() as ModuleConstructor;
                constructorMap.Add(att.type, cons);
            }

            Debug.Log("[Autopilot] load_all_modules_for_vessel constructors.Length = " + constructors.Length.ToString() +
                " constructorMap.Count = " + constructorMap.Keys.Count.ToString());
                
            foreach (Type type in autopilot_module_types)
            {
                if (constructorMap.ContainsKey(type))
                {
                    var constructor = constructorMap[type];
                    load_module(v, type, ve => { return constructor.Invoke(this, new[] { ve }); });
                }
            }
        }

        #region module_constructors

        [ModuleConstructor(typeof(InstantControlModel))]
        InstantControlModel create_InstantControlModel(Vessel v)
        {
            return new InstantControlModel(v);
        }

        [ModuleConstructor(typeof(MediumFlightModel))]
        MediumFlightModel create_MediumFlightModel(Vessel v)
        {
            return new MediumFlightModel(v);
        }

        [ModuleConstructor(typeof(PitchAngularAccController))]
        PitchAngularAccController create_PitchAngularAccController(Vessel v)
        {
            InstantControlModel sfmodel = cur_ves_modules[typeof(InstantControlModel)] as InstantControlModel;
            MediumFlightModel mfmodel = cur_ves_modules[typeof(MediumFlightModel)] as MediumFlightModel;
            return new PitchAngularAccController(v, sfmodel, mfmodel);
        }

        [ModuleConstructor(typeof(PitchAngularVelocityController))]
        PitchAngularVelocityController create_PitchAngularVelocityController(Vessel v)
        {
            InstantControlModel sfmodel = cur_ves_modules[typeof(InstantControlModel)] as InstantControlModel;
            MediumFlightModel mfmodel = cur_ves_modules[typeof(MediumFlightModel)] as MediumFlightModel;
            PitchAngularAccController acc = cur_ves_modules[typeof(PitchAngularAccController)] as PitchAngularAccController;
            return new PitchAngularVelocityController(v, sfmodel, mfmodel, acc);
        }

		[ModuleConstructor(typeof(RollAngularAccController))]
		RollAngularAccController create_RollAngularAccController(Vessel v)
		{
			InstantControlModel sfmodel = cur_ves_modules[typeof(InstantControlModel)] as InstantControlModel;
			MediumFlightModel mfmodel = cur_ves_modules[typeof(MediumFlightModel)] as MediumFlightModel;
			return new RollAngularAccController(v, sfmodel, mfmodel);
		}

		[ModuleConstructor(typeof(RollAngularVelocityController))]
		RollAngularVelocityController create_RollAngularVelocityController(Vessel v)
		{
			InstantControlModel sfmodel = cur_ves_modules[typeof(InstantControlModel)] as InstantControlModel;
			MediumFlightModel mfmodel = cur_ves_modules[typeof(MediumFlightModel)] as MediumFlightModel;
			RollAngularAccController acc = cur_ves_modules[typeof(RollAngularAccController)] as RollAngularAccController;
			return new RollAngularVelocityController(v, sfmodel, mfmodel, acc);
		}

		[ModuleConstructor(typeof(YawAngularAccController))]
		YawAngularAccController create_YawAngularAccController(Vessel v)
		{
			InstantControlModel sfmodel = cur_ves_modules[typeof(InstantControlModel)] as InstantControlModel;
			MediumFlightModel mfmodel = cur_ves_modules[typeof(MediumFlightModel)] as MediumFlightModel;
			return new YawAngularAccController(v, sfmodel, mfmodel);
		}

		[ModuleConstructor(typeof(YawAngularVelocityController))]
		YawAngularVelocityController create_YawAngularVelocityController(Vessel v)
		{
			InstantControlModel sfmodel = cur_ves_modules[typeof(InstantControlModel)] as InstantControlModel;
			MediumFlightModel mfmodel = cur_ves_modules[typeof(MediumFlightModel)] as MediumFlightModel;
			YawAngularAccController acc = cur_ves_modules[typeof(YawAngularAccController)] as YawAngularAccController;
			return new YawAngularVelocityController(v, sfmodel, mfmodel, acc);
		}

        [ModuleConstructor(typeof(TopModuleManager))]
        TopModuleManager create_TopModuleManager(Vessel v)
        {
            MediumFlightModel mfmodel = cur_ves_modules[typeof(MediumFlightModel)] as MediumFlightModel;
            PitchAngularVelocityController pav = cur_ves_modules[typeof(PitchAngularVelocityController)] as PitchAngularVelocityController;
            RollAngularVelocityController rov = cur_ves_modules[typeof(RollAngularVelocityController)] as RollAngularVelocityController;
            YawAngularVelocityController yav = cur_ves_modules[typeof(YawAngularVelocityController)] as YawAngularVelocityController;
            return new TopModuleManager(v, mfmodel, pav, rov, yav);
        }

        #endregion

        private void vesselSwitch(Vessel v)
        {
            Debug.Log("[Autopilot] vessel switch");
            load_all_modules_for_vessel(v);
        }

        ApplicationLauncherButton launcher_btn;

        void onAppLauncherLoad()
        {
            GameEvents.onGUIApplicationLauncherReady.Remove(onAppLauncherLoad);
            launcher_btn = ApplicationLauncher.Instance.AddModApplication(
                OnALTrue, OnALFalse, OnALTrue, OnALUnHover, null, null, ApplicationLauncher.AppScenes.FLIGHT,
                GameDatabase.Instance.GetTexture("AtmosphereAutopilot/icon", false));
        }

        void OnALTrue()
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

        bool styles_init = false;

        public void OnGUI()
        {
            if (!styles_init)
            {
                GUIStyles.Init();
                styles_init = true;
            }
            GUI.skin = GUIStyles.skin;
            applauncher.OnGUI();
            foreach (var pair in cur_ves_modules)
            {
                IAutoGui s = pair.Value as IAutoGui;
                if (s != null)
                    s.OnGUI();
            }
        }

        public void OnHideUI()
        {
            applauncher.HideGUI();
            foreach (var pair in cur_ves_modules)
            {
                IAutoGui s = pair.Value as IAutoGui;
                if (s != null)
                    s.HideGUI();
            }
        }

        public void OnShowUI()
        {
            applauncher.UnHideGUI();
            foreach (var pair in cur_ves_modules)
            {
                IAutoGui s = pair.Value as IAutoGui;
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

            bool mod = GameSettings.MODIFIER_KEY.GetKey();

            foreach (var pair in module_hotkeys)
                if (Input.GetKeyDown(pair.Value))
                {
                    AutopilotModule module = cur_ves_modules[pair.Key] as AutopilotModule;
                    if (module != null)
                    {
                        module.Active = !module.Active;
                        MessageManager.post_status_message(module.ModuleName + (module.Active ? " enabled" : " disabled"));
                    }
                }
        }
    }
}
