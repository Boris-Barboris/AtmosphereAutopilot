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
        ElevatorDamper elevatorDamper;

        public void Start()
        {
            Debug.Log("[Autopilot]: AtmosphereAutopilot starting up!"); 
            DontDestroyOnLoad(this);
            ui_manager = new UI();
            GameEvents.onVesselChange.Add(vesselSwitch);
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
        }

        public void Update()
        {
            if (InputLockManager.IsLocked(ControlTypes.ALL_SHIP_CONTROLS))
                return;
            if (!HighLogic.LoadedSceneIsFlight)
                return;

            if (elevatorDamper != null)
            if (Input.GetKeyDown(KeyCode.P))
            {
                if (GameSettings.MODIFIER_KEY.GetKey())
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
        }
    }
}
