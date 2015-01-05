using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;

namespace AtmosphereAutopilot
{
    /// <summary>
    /// Simple pitch damper on PID
    /// </summary>
    class ElevatorDamper: PIDAutoTrimmer
    {
		public ElevatorDamper(Vessel cur_vessel)
			: base(cur_vessel, "Elevator Dampener", 1238216)
		{ }

        double time = 0.0;
        double regime_start_time = 0.0;
        bool regime = false;
        
        // variablse for trim averaging
        double[] last_output = new double[5];
        int output_i = 0;

        protected override void OnFixedUpdate(FlightCtrlState cntrl)
        {
            angular_velocity = -vessel.angularVelocity.x;
            time = time + TimeWarp.fixedDeltaTime;
            
            // check if user is inputing control
            if (cntrl.killRot)                          // when sas works just back off
                return;
			if (vessel.checkLanded())
            {
                pid.clear();
                regime = false;
                return;
            }
			double raw_control = pid.Control(angular_velocity, 0.0, TimeWarp.fixedDeltaTime);	// get raw control from PID
            if (cntrl.pitch == cntrl.pitchTrim)         // when user doesn't use control, pitch is on the same level as trim
            {
                if (Math.Abs(angular_velocity) < 1e-2)                      // if angular velocity is stabilized
                {
                    if (!regime)
                        regime_start_time = time;
                    regime = true;
                }
                else
                    regime = false;

				output = Common.Clamp(raw_control, 1.0);           // get output from controller
                
                last_output[output_i] = output;                             // register it
                output_i = (output_i + 1) % 5;                              // for averaging

                if (regime && (time - regime_start_time > 1.0))             // if in regime more than 1 second
                    FlightInputHandler.state.pitchTrim = (float)last_output.Average();          // trim
                cntrl.pitch = (float)output;                                // apply output                
            }
            else
            {
				double damper_output = -pid.InputDerivative * pid.KD;
				cntrl.pitch = (float)Common.Clamp(cntrl.pitch + damper_output, 1.0);	// apply damper
                regime = false;
                pid.clear();
                output = damper_output;				
            }
        }
    }
}
