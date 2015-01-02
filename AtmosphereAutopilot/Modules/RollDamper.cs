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
    class RollDamper: PIDAngularVelDampener
    {
        public RollDamper(Vessel cur_vessel)
            : base(cur_vessel, "Roll dampener", 906577) 
        {
            pid = new PIDController();
            pid.KP = 0.1;
            pid.KI = 0.1;
            pid.IntegralClamp = 0.05;
            pid.KD = 0.0001;
        }

        double time = 0.0;

        protected override void OnFixedUpdate(FlightCtrlState cntrl)
        {
            angular_velocity = -currentVessel.angularVelocity.y;
            time = time + TimeWarp.fixedDeltaTime;
            output = pid.Control(angular_velocity, 0.0, time);

            // check if user is inputing control
            if (cntrl.killRot)                          // when sas works just back off
                return;
            if (cntrl.roll == cntrl.rollTrim)           // when user doesn't use control, roll is on the same level as trim
            {
                if (Math.Abs(angular_velocity) < 1e-3)                      // if angular velocity is stabilized
                {
                    FlightInputHandler.state.rollTrim = cntrl.roll;         // trim when necessary
                }
                output = pid.Control(angular_velocity, 0.0, time);          // get output from controller
                cntrl.roll = (float)Common.Clamp(output, 1.0);
            }
            if (currentVessel.checkLanded())
                pid.clear();
        }
    }
}
