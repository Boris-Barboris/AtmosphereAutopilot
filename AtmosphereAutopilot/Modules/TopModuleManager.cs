using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;

namespace AtmosphereAutopilot
{
	public sealed class TopModuleManager : StateController
    {
        AngularVelAdaptiveController[] angular_vc = new AngularVelAdaptiveController[3];

		internal TopModuleManager(Vessel vessel)
			: base(vessel, "Autopilot module manager", 24888888)
		{
			cur_ves_modules = AtmosphereAutopilot.Instance.autopilot_module_lists[vessel];
		}

		Dictionary<Type, AutopilotModule> cur_ves_modules;

        protected override void OnActivate()
        {
			if (cur_ves_modules.Count == 0)
			{
				// We need to create all those modules. Module type needs to define constructor of
				// Constructor(Vessel v) prototype.
				foreach (var module_type in AtmosphereAutopilot.Instance.autopilot_module_types)
				{
					var constructor = module_type.GetConstructor(new[] { typeof(Vessel) });
					cur_ves_modules[module_type] = (AutopilotModule)constructor.Invoke(new[] { vessel });
				}
				// Then we need to resolve relations
				foreach (var module_type in AtmosphereAutopilot.Instance.autopilot_module_types)
					cur_ves_modules[module_type].InitializeDependencies(cur_ves_modules);
			}
			else
			{
				foreach (var module_type in AtmosphereAutopilot.Instance.autopilot_module_types)
					cur_ves_modules[module_type].Activate();
			}

			vessel.OnAutopilotUpdate += new FlightInputCallback(ApplyControl);
        }

        protected override void OnDeactivate()
        {
			foreach (var module_type in AtmosphereAutopilot.Instance.autopilot_module_types)
				cur_ves_modules[module_type].Deactivate();
            vessel.OnAutopilotUpdate -= new FlightInputCallback(ApplyControl);
        }

        public override void ApplyControl(FlightCtrlState state)
        {
            if (vessel.checkLanded())           // ground breaks the model
                return;

            // just set desired angular v to 0.0
            for (int axis = 0; axis < 3; axis++)
            {
                angular_vc[axis].ApplyControl(state, 0.0);
            }
        }
 
    }
}
