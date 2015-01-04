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
            if (scenes == GameScenes.SPACECENTER)
                GUIStyles.Init();
            if (elevatorDamper != null)
                elevatorDamper.serialize();
            if (elevatorDamperEx != null)
                elevatorDamperEx.serialize();
            if (rollDamper != null)
                rollDamper.serialize();
            if (yawDamper != null)
                yawDamper.serialize();
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

            if (!roll_dampers.ContainsKey(v))
            {
                roll_dampers[v] = new RollDamper(v);
                Debug.Log("[Autopilot]: RollDamper for vessel " + v.name + " created");
            }
            else
                Debug.Log("[Autopilot]: RollDamper for vessel " + v.name + " loaded");
            rollDamper = roll_dampers[v];

            if (!yaw_dampers.ContainsKey(v))
            {
                yaw_dampers[v] = new YawDamper(v);
                Debug.Log("[Autopilot]: YawDamper for vessel " + v.name + " created");
            }
            else
                Debug.Log("[Autopilot]: YawDamper for vessel " + v.name + " loaded");
            yawDamper = yaw_dampers[v];

            if (!flight_models.ContainsKey(v))
            {
				flight_models[v] = new InstantControlModel(v);
                Debug.Log("[Autopilot]: FlightModel for vessel " + v.name + " created");
            }
            else
                Debug.Log("[Autopilot]: FlightModel for vessel " + v.name + " loaded");
            flightModel = flight_models[v];

            if (!elevator_dampers_exper.ContainsKey(v))
            {
                elevator_dampers_exper[v] = new ElevatorDamperExperim(v, flightModel);
                Debug.Log("[Autopilot]: ElevatorDamperExperim for vessel " + v.name + " created");
            }
            else
                Debug.Log("[Autopilot]: ElevatorDamperExperim for vessel " + v.name + " loaded");
            elevatorDamperEx = elevator_dampers_exper[v];
        }

        public void OnGUI()
        {
            if (elevatorDamper != null)
                elevatorDamper.drawGUI();
            if (rollDamper != null)
                rollDamper.drawGUI();
            if (yawDamper != null)
                yawDamper.drawGUI();
            if (flightModel != null)
                flightModel.OnGUI();
            if (elevatorDamperEx != null)
                elevatorDamperEx.drawGUI();
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
                        elevatorDamper.toggleGUI();
                    else
                    {
                        if (elevatorDamper.Enabled)
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
                        rollDamper.toggleGUI();
                    else
                    {
                        if (rollDamper.Enabled)
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
                        yawDamper.toggleGUI();
                    else
                    {
                        if (yawDamper.Enabled)
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
                        elevatorDamperEx.toggleGUI();
                    }
                    else
                    {
                        if (elevatorDamperEx.Enabled)
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
