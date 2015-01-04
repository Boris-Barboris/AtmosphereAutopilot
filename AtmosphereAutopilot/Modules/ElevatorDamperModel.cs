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
    class ElevatorDamperExperim: PIDAutoTrimmer
    {
        public ElevatorDamperExperim(Vessel cur_vessel, InstantControlModel model)
            : base(cur_vessel, "Elevator Dampener Experimental", 97756493) 
        {
            this.model = model;
        }

		InstantControlModel model;

        double time = 0.0;
        double regime_start_time = 0.0;
        bool regime = false;
        
        // variablse for trim averaging
        double[] last_output = new double[5];
        int output_i = 0;

		public double MAX_CSURF_SPEED = 6.0;			// maximum control surface speed

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
            double pid_output = pid.Control(angular_velocity, 0.0, time);	// get raw control from PID
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

				double angvd = model.angular_dv[0].getLast();
                double desired_angvd = -model.angular_v[0].getLast() * pid.KP;
                double raw_output = 0.0; // model.get_short_input_for_axis(0, desired_angvd, last_output[output_i]);
                if (double.IsInfinity(raw_output) || double.IsNaN(raw_output))
                    raw_output = last_output[output_i];
                //double smoothed_output = last_output[output_i] + TimeWarp.fixedDeltaTime *
                //    Common.Clamp((raw_output - last_output[output_i]) / TimeWarp.fixedDeltaTime, MAX_CSURF_SPEED);
                double clamped_output = Common.Clamp(raw_output, 1.0);

				output = clamped_output;

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

			output_i = (output_i + 1) % 5;					// for averaging
			last_output[output_i] = cntrl.pitch;            // register output		
        }
    }
}
