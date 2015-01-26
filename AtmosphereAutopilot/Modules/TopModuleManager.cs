using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;
using System.Reflection;

namespace AtmosphereAutopilot
{
	public sealed class TopModuleManager : StateController
    {
		internal TopModuleManager(Vessel vessel)
			: base(vessel, "Autopilot module manager", 24888888)
		{
			cur_ves_modules = AtmosphereAutopilot.Instance.autopilot_module_lists[vessel];
		}

		Dictionary<Type, AutopilotModule> cur_ves_modules;
        List<StateController> HighLevelControllers = new List<StateController>();
        StateController active_controller = null;

        protected override void OnActivate()
        {
            // If this top_manager is the only module loaded for this vessel
			if (cur_ves_modules.Count == 1)
			{
				// We need to create all those modules. Module type needs to define constructor of
				// Constructor(Vessel v) prototype.
                foreach (var module_type in AtmosphereAutopilot.Instance.autopilot_module_types)
				{
                    if (module_type.Equals(typeof(TopModuleManager)))
                        continue;
					var constructor = module_type.GetConstructor(BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance, null, 
                        new[] { typeof(Vessel) }, null);
                    if (constructor == null)
                        throw new NullReferenceException(module_type.Name + " module has no void(Vessel) constructor.");
					cur_ves_modules[module_type] = (AutopilotModule)constructor.Invoke(new[] { vessel });
				}
				// Then we need to resolve relations
                foreach (var module_type in cur_ves_modules.Keys)
                    if (!module_type.Equals(typeof(TopModuleManager)))
                    {
                        cur_ves_modules[module_type].InitializeDependencies(cur_ves_modules);
                        cur_ves_modules[module_type].Deserialize();
                    }

                // Move all high-level controllers to list
                foreach (var module_type in cur_ves_modules.Keys)
                    if (!module_type.Equals(typeof(TopModuleManager)))
                        if (module_type.IsSealed && module_type.IsSubclassOf(typeof(StateController)))
                            HighLevelControllers.Add((StateController)cur_ves_modules[module_type]);

                if (HighLevelControllers.Count <= 0)
                    throw new InvalidOperationException("No high-level autopilot modules were found");
                else
                {
                    active_controller = HighLevelControllers[0];
                    active_controller.Activate();
                }
			}

			vessel.OnAutopilotUpdate += new FlightInputCallback(ApplyControl);
        }

        protected override void OnDeactivate()
        {
            foreach (var module_type in cur_ves_modules.Keys)
                if (!module_type.Equals(typeof(TopModuleManager)))
                {
                    cur_ves_modules[module_type].Serialize();
                    cur_ves_modules[module_type].Deactivate();
                }
            vessel.OnAutopilotUpdate -= new FlightInputCallback(ApplyControl);
        }

        public override void ApplyControl(FlightCtrlState state)
        {
            if (active_controller != null)
                active_controller.ApplyControl(state);
        }

        protected override void _drawGUI(int id)
        {
            GUILayout.BeginVertical();
            Active = GUILayout.Toggle(Active, "MASTER SWITCH", GUIStyles.toggleButtonStyle);
            GUILayout.Space(10);
            foreach (var controller in HighLevelControllers)
            {
                GUILayout.BeginHorizontal();
                bool pressed = GUILayout.Toggle(controller.Active, controller.ModuleName, GUIStyles.toggleButtonStyle);
                if (pressed && !controller.Active)
                {
                    // we activate new module
                    if (active_controller != null)
                        active_controller.Deactivate();
                    controller.Activate();                    
                    active_controller = controller;
                }
                bool is_shown = GUILayout.Toggle(controller.IsShown(), "GUI", GUIStyles.toggleButtonStyle);
                if (is_shown)
                    controller.ShowGUI();
                else
                    controller.UnShowGUI();
                GUILayout.EndHorizontal();
            }
            GUILayout.EndVertical();
            GUI.DragWindow();
        }
 
    }
}
