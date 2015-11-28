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

        [AutoGuiAttr("desired_v", false, "G5")]
        protected float desired_v = 0.0f;

        [AutoGuiAttr("current_v", false, "G5")]
        public double current_v = 0.0f;

        [VesselSerializable("break_margin")]
        [AutoGuiAttr("break spd margin %", true, "G5")]
        public double break_margin = 10.0f;

        [AutoGuiAttr("Use breaks", true)]
        public bool use_breaks = true;

        [AutoGuiAttr("prograde_thrust", false, "G5")]
        public double prograde_thrust;

        //[AutoGuiAttr("drag_estimate", false, "G5")]
        public double drag_estimate;

        [VesselSerializable("Kp_v")]
        [AutoGuiAttr("Kp_v", true, "G5")]
        public double Kp_v = 0.5;

        [AutoGuiAttr("acc_filter_k", true, "G5")]
        public double acc_filter_k = 1.0;

        [AutoGuiAttr("relaxation_acc_error", true, "G5")]
        protected double relaxation_acc_error = 0.1;

        protected Vector3d surfspd_dir;

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
                surfspd_dir = imodel.surface_v.normalized;
                Vector3 thrust = imodel.cntrl_part_to_world * imodel.engines_thrust_principal;
                prograde_thrust = Vector3.Dot(thrust, surfspd_dir);

                double current_acc = Vector3.Dot(imodel.sum_acc, imodel.prev_surface_v);
                drag_estimate = current_acc - prograde_thrust / imodel.sum_mass - Vector3d.Dot(imodel.gravity_acc, surfspd_dir) -
                    Vector3d.Dot(imodel.noninert_acc, surfspd_dir);

                v_error = desired_v - current_v;
                desired_acc = Kp_v * v_error;
                if (Math.Abs(desired_acc - current_acc) < relaxation_acc_error)
                {
                    // we're on low error regime, let's smooth out acceleration error using exponential moving average
                    acc_error = Common.simple_filter(desired_acc - current_acc, acc_error, acc_filter_k);
                }
                else
                {
                    acc_error = desired_acc - current_acc;              
                }

                thrust_error = acc_error * imodel.sum_mass;
                cntrl.mainThrottle = solve_thrust_req(prograde_thrust + thrust_error, prev_input);

                prev_thrust = prograde_thrust;
            }
            
            prev_input = cntrl.mainThrottle;
            return cntrl.mainThrottle;
        }

        double prev_thrust;
        float prev_input;

        int[] throttle_directions;
        double[] estimated_max_thrusts;

        float solve_thrust_req(double required_thrust, float prev_throttle)
        {
            if (imodel.engines.Count == 0)
                return 0.0f;

            Common.Realloc(ref throttle_directions, imodel.engines.Count);
            Common.Realloc(ref estimated_max_thrusts, imodel.engines.Count);

            double predicted_thrust = 0.0;
            for (int i = 0; i < imodel.engines.Count; i++)
            {
                ModuleEngines eng = imodel.engines[i].engine;
                double e_prograde_thrust = Vector3.Dot(imodel.engines[i].thrust, surfspd_dir);
                double e_throttle = eng.currentThrottle / eng.thrustPercentage * 100.0;
                estimated_max_thrusts[i] = 
                    e_prograde_thrust != 0.0 ? e_prograde_thrust / e_throttle : eng.maxThrust;
                if (eng.useEngineResponseTime)
                {
                    throttle_directions[i] = prev_throttle >= eng.currentThrottle ? 1 : -1;
                    if (Mathf.Abs(eng.currentThrottle - prev_throttle * eng.thrustPercentage * 0.01f) <= 1e-4f)
                    {
                        throttle_directions[i] = 0;
                        predicted_thrust += estimated_max_thrusts[i] * prev_throttle;
                    }
                    else
                    {
                        float spd = throttle_directions[i] > 0 ? eng.engineAccelerationSpeed : eng.engineDecelerationSpeed;
                        double predict_throttle = Mathf.Lerp(eng.currentThrottle, prev_throttle, TimeWarp.fixedDeltaTime * spd);
                        predicted_thrust += estimated_max_thrusts[i] * predict_throttle;
                    }
                }
                else
                    predicted_thrust += estimated_max_thrusts[i] * prev_throttle;
            }

            bool spool_dir_changed = false;
            double desired_throttle = 0.0;
            double t_error = required_thrust - predicted_thrust;
            iter_count = 0;
            do
            {
                spool_dir_changed = false;                
                double thrust_authority = 0.0;
                for (int i = 0; i < imodel.engines.Count; i++)
                {
                    ModuleEngines eng = imodel.engines[i].engine;
                    if (eng.useEngineResponseTime && throttle_directions[i] != 0)
                    {
                        float spd = throttle_directions[i] > 0 ? eng.engineAccelerationSpeed : eng.engineDecelerationSpeed;
                        thrust_authority += TimeWarp.fixedDeltaTime * spd * estimated_max_thrusts[i];
                    }
                    else
                        thrust_authority += estimated_max_thrusts[i];
                }
                if (thrust_authority == 0.0)
                    return 0.0f;
                desired_throttle = Common.Clamp(prev_throttle + t_error / thrust_authority, 0.0, 1.0);

                // now check if we changed spooling direction
                for (int i = 0; i < imodel.engines.Count; i++)
                {
                    ModuleEngines eng = imodel.engines[i].engine;
                    if (eng.useEngineResponseTime)
                    {
                        int new_throttle_direction = 0;
                        if (Math.Abs(eng.currentThrottle - desired_throttle * eng.thrustPercentage * 0.01) > 1e-4)
                            new_throttle_direction = (desired_throttle > (eng.currentThrottle / eng.thrustPercentage * 100)) ? 1 : -1;
                        if (throttle_directions[i] != new_throttle_direction)
                        {
                            spool_dir_changed = true;
                            throttle_directions[i] = new_throttle_direction;
                        }
                    }
                }
                iter_count++;
            } while (spool_dir_changed && iter_count <= imodel.engines.Count + 2);

            return (float)Common.Clamp(desired_throttle, 0.0, 1.0);
        }

        [AutoGuiAttr("iter_count", false)]
        protected int iter_count;

        [AutoGuiAttr("v_error", false, "G5")]
        double v_error;

        [AutoGuiAttr("desired_acc", false, "G5")]
        double desired_acc;

        [AutoGuiAttr("acc_error", false, "G5")]
        double acc_error;

        [AutoGuiAttr("thrust_error", false, "G5")]
        double thrust_error;

        [VesselSerializable("use_pid")]
        [AutoGuiAttr("Use PID", true)]
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
