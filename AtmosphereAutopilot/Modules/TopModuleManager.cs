/*
Atmosphere Autopilot, plugin for Kerbal Space Program.
Copyright (C) 2015, Baranin Alexander aka Boris-Barboris.
 
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
	public sealed class TopModuleManager : StateController
    {
		internal TopModuleManager(Vessel vessel)
			: base(vessel, "Autopilot module manager", 24888888)
		{
			cur_ves_modules = AtmosphereAutopilot.Instance.autopilot_module_lists[vessel];
		}

        // All finished autopilot modules, created for this vessel
		Dictionary<Type, AutopilotModule> cur_ves_modules;

        // All high-level autopilot modules, created for this vessel
        Dictionary<Type, StateController> HighLevelControllers = new Dictionary<Type, StateController>();

        // Currently active high-level autopilot
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
				// Then we need to resolve relations and deserialize
                foreach (var module_type in cur_ves_modules.Keys)
                    if (!module_type.Equals(typeof(TopModuleManager)))
                    {
                        cur_ves_modules[module_type].InitializeDependencies(cur_ves_modules);
                        cur_ves_modules[module_type].Deserialize();
                    }

                // Move all high-level controllers to list
                foreach (var module_type in cur_ves_modules.Keys)
                    if (!module_type.Equals(typeof(TopModuleManager)))
                        if (module_type.IsSubclassOf(typeof(StateController)))
                            HighLevelControllers.Add(module_type, (StateController)cur_ves_modules[module_type]);

                if (HighLevelControllers.Count <= 0)
                    throw new InvalidOperationException("No high-level autopilot modules were found");
                else
                    active_controller = HighLevelControllers[typeof(StandardFlyByWire)];
			}

            if (active_controller != null)
                active_controller.Activate();
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
            foreach (var controller in HighLevelControllers.Values)
            {
                GUILayout.BeginHorizontal();
                bool pressed = GUILayout.Toggle(controller.Active, controller.ModuleName, GUIStyles.toggleButtonStyle);
                if (pressed && !controller.Active && Active)
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
