using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;

namespace AtmosphereAutopilot
{
    /// <summary>
    /// Simple roll damper on PID
    /// </summary>
    class RollDamper: PIDAutoTrimmer
    {
		public RollDamper(Vessel cur_vessel)
			: base(cur_vessel, "Roll Dampener", 906577)
		{ }

        double time = 0.0;

        protected override void OnFixedUpdate(FlightCtrlState cntrl)
        {
            angular_velocity = -vessel.angularVelocity.y;
            time = time + TimeWarp.fixedDeltaTime;

            // check if user is inputing control
            if (cntrl.killRot)                          // when sas works just back off
                return;
			if (vessel.checkLanded())
            {
                pid.clear();
                return;
            }
			output = pid.Control(angular_velocity, 0.0, time);				// get output from controller
            if (cntrl.roll == cntrl.rollTrim)           // when user doesn't use control, roll is on the same level as trim
            {
                cntrl.roll = (float)Common.Clamp(output, 1.0);
                if (Math.Abs(angular_velocity) < 1e-3)                      // if angular velocity is stabilized
                {
                    FlightInputHandler.state.rollTrim = cntrl.roll;         // trim when necessary
                }
            }
            else
            {
				double damper_output = -pid.InputDerivative * pid.KD;
				cntrl.pitch = (float)Common.Clamp(cntrl.pitch + damper_output, 1.0);	// apply damper
                pid.clear();
                output = damper_output;
            }
        }
    }
}
