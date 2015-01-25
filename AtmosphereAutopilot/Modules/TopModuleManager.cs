using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;

namespace AtmosphereAutopilot
{
    public sealed class TopModuleManager : AutopilotModule
    {
        MediumFlightModel mmodel;
        AngularVelAdaptiveController[] angular_vc = new AngularVelAdaptiveController[3];

        internal TopModuleManager(Vessel vessel, MediumFlightModel mmodel, PitchAngularVelocityController pvc,
            RollAngularVelocityController rvc, YawAngularVelocityController yvc)
            : base(vessel, 24888888, "Autopilot panel")
        {
            this.mmodel = mmodel;
            angular_vc[PITCH] = pvc;
            angular_vc[ROLL] = rvc;
            angular_vc[YAW] = yvc;
        }

        protected override void OnActivate()
        {
            vessel.OnAutopilotUpdate += new FlightInputCallback(ApplyControl);
        }

        protected override void OnDeactivate()
        {
            vessel.OnAutopilotUpdate -= new FlightInputCallback(ApplyControl);
        }

        public void ApplyControl(FlightCtrlState state)
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
