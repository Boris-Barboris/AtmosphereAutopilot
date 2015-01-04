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
        Dictionary<Vessel, ElevatorDamper> elevator_dampers = new Dictionary<Vessel, ElevatorDamper>();
        Dictionary<Vessel, RollDamper> roll_dampers = new Dictionary<Vessel, RollDamper>();
        Dictionary<Vessel, YawDamper> yaw_dampers = new Dictionary<Vessel, YawDamper>();
		Dictionary<Vessel, InstantControlModel> flight_models = new Dictionary<Vessel, InstantControlModel>();
        Dictionary<Vessel, ElevatorDamperExperim> elevator_dampers_exper = new Dictionary<Vessel, ElevatorDamperExperim>();
        ElevatorDamper elevatorDamper;
        ElevatorDamperExperim elevatorDamperEx;
        RollDamper rollDamper;
        YawDamper yawDamper;
        InstantControlModel flightModel;

        public static AtmosphereAutopilot Instance { get; private set; }

        public void Start()
        {
            Debug.Log("[Autopilot]: AtmosphereAutopilot starting up!"); 
            DontDestroyOnLoad(this);
            GameEvents.onVesselChange.Add(vesselSwitch);
            GameEvents.onGameSceneLoadRequested.Add(sceneSwitch);
            Instance = this;
        }

        public void OnDestroy()
        {
            sceneSwitch(GameScenes.CREDITS);
        }

        private void sceneSwitch(GameScenes scenes)
        {
            if (elevatorDamper != null)
                elevatorDamper.Serialize();
            if (elevatorDamperEx != null)
				elevatorDamperEx.Serialize();
            if (rollDamper != null)
				rollDamper.Serialize();
            if (yawDamper != null)
				yawDamper.Serialize();
            if (flightModel != null)
                flightModel.Serialize();
            elevatorDamper = null; 
            elevatorDamperEx = null;
            rollDamper = null;
            yawDamper = null;
            flightModel = null;
        }

        private void vesselSwitch(Vessel v)
        {
            if (!elevator_dampers.ContainsKey(v))
            {
                elevator_dampers[v] = new ElevatorDamper(v);
                Debug.Log("[Autopilot]: ElevatorDamper for vessel " + v.name + " created");
            }
            else
                Debug.Log("[Autopilot]: ElevatorDamper for vessel " + v.name + " loaded");
            elevatorDamper = elevator_dampers[v];
            elevatorDamper.Deserialize();

            if (!roll_dampers.ContainsKey(v))
            {
                roll_dampers[v] = new RollDamper(v);
                Debug.Log("[Autopilot]: RollDamper for vessel " + v.name + " created");
            }
            else
                Debug.Log("[Autopilot]: RollDamper for vessel " + v.name + " loaded");
            rollDamper = roll_dampers[v];
            rollDamper.Deserialize();

            if (!yaw_dampers.ContainsKey(v))
            {
                yaw_dampers[v] = new YawDamper(v);
                Debug.Log("[Autopilot]: YawDamper for vessel " + v.name + " created");
            }
            else
                Debug.Log("[Autopilot]: YawDamper for vessel " + v.name + " loaded");
            yawDamper = yaw_dampers[v];
            yawDamper.Deserialize();

            if (!flight_models.ContainsKey(v))
            {
				flight_models[v] = new InstantControlModel(v);
                Debug.Log("[Autopilot]: FlightModel for vessel " + v.name + " created");
            }
            else
                Debug.Log("[Autopilot]: FlightModel for vessel " + v.name + " loaded");
            flightModel = flight_models[v];
            flightModel.Deserialize();

            if (!elevator_dampers_exper.ContainsKey(v))
            {
                elevator_dampers_exper[v] = new ElevatorDamperExperim(v, flightModel);
                Debug.Log("[Autopilot]: ElevatorDamperExperim for vessel " + v.name + " created");
            }
            else
                Debug.Log("[Autopilot]: ElevatorDamperExperim for vessel " + v.name + " loaded");
            elevatorDamperEx = elevator_dampers_exper[v];
            elevatorDamperEx.Deserialize();
        }

        bool styles_init = false;

        public void OnGUI()
        {
            if (!styles_init)
            {
                GUIStyles.Init();
                styles_init = true;
            }
            if (elevatorDamper != null)
                elevatorDamper.OnGUI();
            if (rollDamper != null)
				rollDamper.OnGUI();
            if (yawDamper != null)
				yawDamper.OnGUI();
            if (flightModel != null)
                flightModel.OnGUI();
            if (elevatorDamperEx != null)
				elevatorDamperEx.OnGUI();
        }

        public void Update()
        {
            if (InputLockManager.IsLocked(ControlTypes.ALL_SHIP_CONTROLS))
                return;
            if (!HighLogic.LoadedSceneIsFlight)
                return;

            bool mod = GameSettings.MODIFIER_KEY.GetKey();

            if (elevatorDamper != null)
                if (Input.GetKeyDown(KeyCode.P))
                {
                    if (mod)
                        elevatorDamper.ToggleGUI();
                    else
                    {
                        if (elevatorDamper.Active)
                        {
                            elevatorDamper.Deactivate();
                            MessageManager.post_status_message("Elevator damper disabled");
                        }
                        else
                        {
                            elevatorDamper.Activate();
                            MessageManager.post_status_message("Elevator damper enabled");
                        }
                    }
                }

            if (rollDamper != null)
                if (Input.GetKeyDown(KeyCode.O))
                {
                    if (mod)
                        rollDamper.ToggleGUI();
                    else
                    {
						if (rollDamper.Active)
                        {
                            rollDamper.Deactivate();
                            MessageManager.post_status_message("Roll damper disabled");
                        }
                        else
                        {
                            rollDamper.Activate();
                            MessageManager.post_status_message("Roll damper enabled");
                        }
                    }
                }

            if (yawDamper != null)
                if (Input.GetKeyDown(KeyCode.Slash))
                {
                    if (mod)
                        yawDamper.ToggleGUI();
                    else
                    {
						if (yawDamper.Active)
                        {
                            yawDamper.Deactivate();
                            MessageManager.post_status_message("Yaw damper disabled");
                        }
                        else
                        {
                            yawDamper.Activate();
                            MessageManager.post_status_message("Yaw damper enabled");
                        }
                    }
                }

            if (flightModel != null && elevatorDamperEx != null)
                if (Input.GetKeyDown(KeyCode.F7))
                {
                    if (mod)
                    {
                        flightModel.ToggleGUI();
                        elevatorDamperEx.ToggleGUI();
                    }
                    else
                    {
						if (elevatorDamperEx.Active)
                        {
                            elevatorDamperEx.Deactivate();
                            MessageManager.post_status_message("elevatorDamperEx damper disabled");
                        }
                        else
                        {
                            elevatorDamperEx.Activate();
                            MessageManager.post_status_message("elevatorDamperEx damper enabled");
                        }
                    }
                }
        }
    }
}
