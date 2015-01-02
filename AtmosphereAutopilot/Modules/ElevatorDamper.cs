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
            pid.KP = 3.0;
            pid.KI = 10.0;
            pid.AccumulatorClamp = 0.1;
            pid.AccumulDerivClamp = 0.033;
            pid.KD = 0.25;
            pid.IntegralClamp = 0.4;
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
                    regime = true;
                }

                output = Common.Clamp(pid.Control(angular_velocity, 0.0, time), 1.0);           // get output from controller
                
                last_output[output_i] = output;                             // register it
                output_i = (output_i + 1) % 5;                              // for averaging

                if (regime)
                    FlightInputHandler.state.pitchTrim = (float)last_output.Average();          // trim
                cntrl.pitch = (float)output;                                // apply output                
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
