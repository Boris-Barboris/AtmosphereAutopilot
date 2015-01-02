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
        UI ui_manager = null;
        Dictionary<Vessel, ElevatorDamper> elevator_dampers = new Dictionary<Vessel, ElevatorDamper>();
        Dictionary<Vessel, RollDamper> roll_dampers = new Dictionary<Vessel, RollDamper>();
        Dictionary<Vessel, YawDamper> yaw_dampers = new Dictionary<Vessel, YawDamper>();
        ElevatorDamper elevatorDamper;
        RollDamper rollDamper;
        YawDamper yawDamper;

        public void Start()
        {
            Debug.Log("[Autopilot]: AtmosphereAutopilot starting up!"); 
            DontDestroyOnLoad(this);
            ui_manager = new UI();
            GameEvents.onVesselChange.Add(vesselSwitch);
            GameEvents.onGameSceneLoadRequested.Add(sceneSwitch);
        }

        public void OnDestroy()
        {
            sceneSwitch(GameScenes.CREDITS);
        }

        private void sceneSwitch(GameScenes scenes)
        {
            if (elevatorDamper != null)
                elevatorDamper.serialize();
            if (rollDamper != null)
                rollDamper.serialize();
            if (yawDamper != null)
                yawDamper.serialize();
            elevatorDamper = null;
            rollDamper = null;
            yawDamper = null;
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
                            ui_manager.post_status_message("Elevator damper disabled");
                        }
                        else
                        {
                            elevatorDamper.Activate();
                            ui_manager.post_status_message("Elevator damper enabled");
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
                            ui_manager.post_status_message("Roll damper disabled");
                        }
                        else
                        {
                            rollDamper.Activate();
                            ui_manager.post_status_message("Roll damper enabled");
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
                            ui_manager.post_status_message("Yaw damper disabled");
                        }
                        else
                        {
                            yawDamper.Activate();
                            ui_manager.post_status_message("Yaw damper enabled");
                        }
                    }
                }
        }
    }
}
