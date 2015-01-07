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
        Dictionary<Type, object> active_modules = new Dictionary<Type, object>();
        Dictionary<Type, KeyCode> module_hotkeys = new Dictionary<Type, KeyCode>();

        public static AtmosphereAutopilot Instance { get; private set; }

        public void Start()
        {
            Debug.Log("[Autopilot]: AtmosphereAutopilot starting up!"); 
            DontDestroyOnLoad(this);
            initialize_types();
            initialize_module_lists();
            initialize_hotkeys();
            GameEvents.onVesselChange.Add(vesselSwitch);
            GameEvents.onGameSceneLoadRequested.Add(sceneSwitch);
            GameEvents.onHideUI.Add(OnHideUI);
            GameEvents.onShowUI.Add(OnShowUI);
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
        }

        void initialize_hotkeys()
        {
            module_hotkeys[typeof(InstantControlModel)] = KeyCode.Alpha8;
            module_hotkeys[typeof(MediumFlightModel)] = KeyCode.Alpha9;
            module_hotkeys[typeof(PitchAngularAccController)] = KeyCode.P;
            module_hotkeys[typeof(PitchAngularVelocityController)] = KeyCode.P;
			module_hotkeys[typeof(RollAngularAccController)] = KeyCode.O;
			module_hotkeys[typeof(RollAngularVelocityController)] = KeyCode.O;
			module_hotkeys[typeof(YawAngularAccController)] = KeyCode.Slash;
			module_hotkeys[typeof(YawAngularVelocityController)] = KeyCode.Slash;
        }

        void initialize_module_lists()
        {
            foreach (Type type in autopilot_module_types)
                autopilot_module_lists[type] = new Dictionary<Vessel, object>();
        }

        void serialize_active_modules()
        {
            foreach (var pair in active_modules)
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
            active_modules.Clear();
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
            active_modules[type] = module;
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
            InstantControlModel sfmodel = active_modules[typeof(InstantControlModel)] as InstantControlModel;
            MediumFlightModel mfmodel = active_modules[typeof(MediumFlightModel)] as MediumFlightModel;
            return new PitchAngularAccController(v, sfmodel, mfmodel);
        }

        [ModuleConstructor(typeof(PitchAngularVelocityController))]
        PitchAngularVelocityController create_PitchAngularVelocityController(Vessel v)
        {
            InstantControlModel sfmodel = active_modules[typeof(InstantControlModel)] as InstantControlModel;
            MediumFlightModel mfmodel = active_modules[typeof(MediumFlightModel)] as MediumFlightModel;
            PitchAngularAccController acc = active_modules[typeof(PitchAngularAccController)] as PitchAngularAccController;
            return new PitchAngularVelocityController(v, sfmodel, mfmodel, acc);
        }

		[ModuleConstructor(typeof(RollAngularAccController))]
		RollAngularAccController create_RollAngularAccController(Vessel v)
		{
			InstantControlModel sfmodel = active_modules[typeof(InstantControlModel)] as InstantControlModel;
			MediumFlightModel mfmodel = active_modules[typeof(MediumFlightModel)] as MediumFlightModel;
			return new RollAngularAccController(v, sfmodel, mfmodel);
		}

		[ModuleConstructor(typeof(RollAngularVelocityController))]
		RollAngularVelocityController create_RollAngularVelocityController(Vessel v)
		{
			InstantControlModel sfmodel = active_modules[typeof(InstantControlModel)] as InstantControlModel;
			MediumFlightModel mfmodel = active_modules[typeof(MediumFlightModel)] as MediumFlightModel;
			RollAngularAccController acc = active_modules[typeof(RollAngularAccController)] as RollAngularAccController;
			return new RollAngularVelocityController(v, sfmodel, mfmodel, acc);
		}

		[ModuleConstructor(typeof(YawAngularAccController))]
		YawAngularAccController create_YawAngularAccController(Vessel v)
		{
			InstantControlModel sfmodel = active_modules[typeof(InstantControlModel)] as InstantControlModel;
			MediumFlightModel mfmodel = active_modules[typeof(MediumFlightModel)] as MediumFlightModel;
			return new YawAngularAccController(v, sfmodel, mfmodel);
		}

		[ModuleConstructor(typeof(YawAngularVelocityController))]
		YawAngularVelocityController create_YawAngularVelocityController(Vessel v)
		{
			InstantControlModel sfmodel = active_modules[typeof(InstantControlModel)] as InstantControlModel;
			MediumFlightModel mfmodel = active_modules[typeof(MediumFlightModel)] as MediumFlightModel;
			YawAngularAccController acc = active_modules[typeof(YawAngularAccController)] as YawAngularAccController;
			return new YawAngularVelocityController(v, sfmodel, mfmodel, acc);
		}

        [ModuleConstructor(typeof(RollDamper))]
        RollDamper create_RollDamper(Vessel v)
        {
            return new RollDamper(v);
        }

        [ModuleConstructor(typeof(YawDamper))]
        YawDamper create_YawDamper(Vessel v)
        {
            return new YawDamper(v);
        }

        #endregion

        private void vesselSwitch(Vessel v)
        {
            Debug.Log("[Autopilot] vessel switch");
            load_all_modules_for_vessel(v);
        }

        bool styles_init = false;

        public void OnGUI()
        {
            if (!styles_init)
            {
                GUIStyles.Init();
                styles_init = true;
            }
            foreach (var pair in active_modules)
            {
                IAutoGui s = pair.Value as IAutoGui;
                if (s != null)
                    s.OnGUI();
            }
        }

        public void OnHideUI()
        {
            foreach (var pair in active_modules)
            {
                IAutoGui s = pair.Value as IAutoGui;
                if (s != null)
                    s.HideGUI();
            }
        }

        public void OnShowUI()
        {
            foreach (var pair in active_modules)
            {
                IAutoGui s = pair.Value as IAutoGui;
                if (s != null)
                    s.ShowGUI();
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
                    if (mod)
                    {
                        IAutoGui gui = active_modules[pair.Key] as IAutoGui;
                        if (gui != null)
                            gui.ToggleGUI();
                    }
                    else
                    {
                        AutopilotModule module = active_modules[pair.Key] as AutopilotModule;
                        if (module != null)
                        {
                            module.Active = !module.Active;
                            MessageManager.post_status_message(module.ModuleName + (module.Active ? " enabled" : " disabled"));
                        }
                    }
        }
    }
}
