using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;

namespace AtmosphereAutopilot
{
    /// <summary>
    /// Simple yaw damper on PID
    /// </summary>
    class YawDamper: PIDAutoTrimmer
    {
		public YawDamper(Vessel cur_vessel)
			: base(cur_vessel, "Yaw Dampener", 752348)
		{ }

        double time = 0.0;

        protected override void OnFixedUpdate(FlightCtrlState cntrl)
        {
            angular_velocity = -vessel.angularVelocity.z;
            time = time + TimeWarp.fixedDeltaTime;
            
            // check if user is inputing control
            if (cntrl.killRot)                          // when sas works just back off
                return;
            if (cntrl.yaw == cntrl.yawTrim)             // when user doesn't use control, yaw is on the same level as trim
            {
				output = pid.Control(angular_velocity, 0.0, TimeWarp.fixedDeltaTime);          // get output from controller
                cntrl.yaw = (float)Common.Clamp(output, 1.0);
            }
            else
            {
                pid.clear();
                output = 0.0;
            }
        }
    }
}
