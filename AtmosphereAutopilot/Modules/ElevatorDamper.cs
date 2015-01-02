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
    class ElevatorDamper: PIDAngularVelDampener
    {
        public ElevatorDamper(Vessel cur_vessel)
            : base(cur_vessel, "Elevator dampener", 1238216) 
        {
            pid = new PIDController();
            pid.KP = 1.0;
            pid.KI = 3.0;
            pid.IntegralClamp = 0.33;
            pid.KD = 0.001;
        }

        double time = 0.0;
        bool regime = false;
        
        // variablse for trim averaging
        double[] last_output = new double[5];
        int output_i = 0;

        protected override void OnFixedUpdate(FlightCtrlState cntrl)
        {
            angular_velocity = -currentVessel.angularVelocity.x;
            time = time + TimeWarp.fixedDeltaTime;
            
            // check if user is inputing control
            if (cntrl.killRot)                          // when sas works just back off
                return;
            if (currentVessel.checkLanded())
            {
                pid.clear();
                regime = false;
                return;
            }
            if (cntrl.pitch == cntrl.pitchTrim)         // when user doesn't use control, pitch is on the same level as trim
            {
                if (Math.Abs(angular_velocity) < 1e-3)                      // if angular velocity is stabilized
                {
                    if (!regime)
                        for (int i = 0; i < 5; i++)
                            last_output[i] = output;
                    regime = true;
                }
                
                output = pid.Control(angular_velocity, 0.0, time);          // get output from controller
                
                last_output[output_i] = output;                             // register it
                output_i = (output_i + 1) % 5;                              // for averaging
                
                cntrl.pitch = (float)Common.Clamp(output, 1.0);             // apply output
                if (regime)
                    FlightInputHandler.state.pitchTrim = (float)last_output.Average();      // trim
            }
            else
            {
                regime = false;
                pid.clear();
                output = 0.0;
            }
        }
    }
}
