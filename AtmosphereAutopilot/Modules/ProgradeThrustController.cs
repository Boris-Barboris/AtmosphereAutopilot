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

namespace AtmosphereAutopilot
{

    /// <summary>
    /// Naive thrust controller for regular plane flight.
    /// </summary>
    public sealed class ProgradeThrustController : SISOController
    {
        FlightModel imodel;

        internal ProgradeThrustController(Vessel v)
            : base(v, "Prograde thrust controller", 88437224)
        {
            pid.IntegralClamp = double.PositiveInfinity;
            pid.AccumulDerivClamp = double.PositiveInfinity;
            pid.KP = 0.4;
            pid.KI = 0.1;
            pid.KD = 0.5;
        }

        public override void InitializeDependencies(Dictionary<Type, AutopilotModule> modules)
        {
            imodel = modules[typeof(FlightModel)] as FlightModel;
        }

        protected override void OnActivate()
        {
            imodel.Activate();
        }

        protected override void OnDeactivate()
        {
            imodel.Deactivate();
        }

        [AutoGuiAttr("desired_v", false, "G4")]
        protected float desired_v = 0.0f;

        [AutoGuiAttr("current_v", false, "G5")]
        public double current_v = 0.0f;

        [VesselSerializable("break_margin")]
        [AutoGuiAttr("break spd margin %", true, "G4")]
        public double break_margin = 10.0f;

        [AutoGuiAttr("Use breaks", true)]
        public bool use_breaks = true;

        [AutoGuiAttr("prograde_thrust", false, "G5")]
        public double prograde_thrust;

        [AutoGuiAttr("drag_estimate", false, "G5")]
        public double drag_estimate;

        [VesselSerializable("Kp_v")]
        [AutoGuiAttr("Kp_v", true, "G4")]
        public double Kp_v = 50.0;

        [AutoGuiAttr("Ki_throttle", true, "G4")]
        public double Ki_throttle = 0.1;

        [AutoGuiAttr("Kd_throttle", true, "G4")]
        public double Kd_throttle = 0.1;

        [AutoGuiAttr("relaxation_throttle", true, "G4")]
        public double relaxation_throttle = 0.1;

        /// <summary>
        /// Main control function
        /// </summary>
        /// <param name="cntrl">Control state to change</param>
        /// <param name="target_value">Prograde surface speed setpoint</param>
        public override float ApplyControl(FlightCtrlState cntrl, float target_value)
        {
            current_v = imodel.surface_v.magnitude;
            desired_v = target_value;

            // apply breaks if needed
            if (use_breaks)
            {
                if (vessel.LandedOrSplashed)
                {
                    // we're on ground
                    if (current_v > desired_v)
                        vessel.ActionGroups.SetGroup(KSPActionGroup.Brakes, true);
                    else
                        vessel.ActionGroups.SetGroup(KSPActionGroup.Brakes, false);
                }
                else
                {
                    // we're in flight
                    if (current_v > (1.0 + break_margin / 100.0) * desired_v)
                        vessel.ActionGroups.SetGroup(KSPActionGroup.Brakes, true);
                    else
                        vessel.ActionGroups.SetGroup(KSPActionGroup.Brakes, false);
                }
            }

            if (use_pid)
            {
                if (pid.KI != 0.0)
                {
                    if (pid.KP != 0.0)
                        pid.IntegralClamp = 1.0 / pid.KP;
                    pid.AccumulatorClamp = 1.0 / pid.KI;
                }
                cntrl.mainThrottle = Mathf.Clamp01((float)pid.Control(current_v, desired_v, TimeWarp.fixedDeltaTime));
            }
            else
            {
                Vector3 thrust = imodel.cntrl_part_to_world * imodel.engines_thrust_principal;
                prograde_thrust = Vector3.Dot(thrust, imodel.surface_v);

                double current_acc = Vector3.Dot(imodel.sum_acc - imodel.gravity_acc - imodel.noninert_acc, imodel.surface_v);
                if (current_v > 5.0)
                    drag_estimate = current_acc - prograde_thrust / imodel.sum_mass;
                else
                    drag_estimate = 0.0;

                v_error = desired_v - current_v;
                desired_acc = Kp_v * v_error;
                acc_error = desired_acc - current_acc;
                thrust_error = acc_error * imodel.sum_mass;

                dthrust_dt = (prograde_thrust - prev_thrust) / TimeWarp.fixedDeltaTime;
                dinput_dt = (prev_input - prev2_input) / TimeWarp.fixedDeltaTime;

                if (prograde_thrust < -0.1)
                    cntrl.mainThrottle = 0.0f;
                else
                {
                    //kti = dthrust_dt / dinput_dt;
                    //if (double.IsNaN(kti) || double.IsInfinity(kti) || kti < 0.0)
                    //{
                    kti = imodel.engines.Sum(m => m.engine.maxThrust * m.engine.thrustPercentage);
                    //}
                    double kd = Math.Abs(thrust_error) < relaxation_throttle * kti ? Kd_throttle : 1.0;
                    cntrl.mainThrottle = Mathf.Clamp01((float)(prev_input + Ki_throttle * kd * thrust_error / kti));
                }

                prev_thrust = prograde_thrust;
            }
            
            prev2_input = prev_input;
            prev_input = cntrl.mainThrottle;
            return cntrl.mainThrottle;
        }

        double prev_thrust;
        double prev_input, prev2_input;

        [AutoGuiAttr("v_error", false, "G4")]
        double v_error;

        [AutoGuiAttr("desired_acc", false, "G4")]
        double desired_acc;

        [AutoGuiAttr("acc_error", false, "G4")]
        double acc_error;

        [AutoGuiAttr("thrust_error", false, "G4")]
        double thrust_error;

        [AutoGuiAttr("dthrust_dt", false, "G4")]
        double dthrust_dt;

        [AutoGuiAttr("dinput_dt", false, "G4")]
        double dinput_dt;

        [AutoGuiAttr("kti", false, "G6")]
        double kti;

        [VesselSerializable("use_pid")]
        [AutoGuiAttr("Use pid", true)]
        public bool use_pid = false;

        PIDController pid = new PIDController();

        [VesselSerializable("pid_Kp")]
        [AutoGuiAttr("pid_Kp", true, "G4")]
        public double pid_Kp { get { return pid.KP; } set { pid.KP = value; } }

        [VesselSerializable("pid_Ki")]
        [AutoGuiAttr("pid_Ki", true, "G4")]
        public double pid_Ki { get { return pid.KI; } set { pid.KI = value; } }

        [VesselSerializable("pid_Kd")]
        [AutoGuiAttr("pid_Kd", true, "G4")]
        public double pid_Kd { get { return pid.KD; } set { pid.KD = value; } }

    }
}
