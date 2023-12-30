﻿/*
Atmosphere Autopilot, plugin for Kerbal Space Program.
Copyright (C) 2015-2016, Baranin Alexander aka Boris-Barboris.

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
    public enum SpeedType
    {
        MetersPerSecond,
        Knots,
        Mach,
        IAS,
        KIAS
    }

    public static class SpeedTypeMethods
    {
        // SpeedType classification
        public static bool isKinematicSystem(this SpeedType t)
        {
            switch (t)
            {
                case SpeedType.MetersPerSecond:
                case SpeedType.Knots:
                case SpeedType.Mach:
                    return true;
                case SpeedType.IAS:
                case SpeedType.KIAS:
                    return false;
                default:
                    throw new NotImplementedException();
            }
        }

        public static bool isPressureSystem(this SpeedType t)
        {
            return !isKinematicSystem(t);
        }

        public static bool isSameSystem(this SpeedType t1, SpeedType t2)
        {
            return isKinematicSystem(t1) == isKinematicSystem(t2);
        }
    }

    public struct SpeedSetpoint
    {
        public SpeedType type;
        public float value;
        Vessel v;

        public const float kts2mps = 0.514444f;
        public const float mps2kts = 1.0f / kts2mps;

        public SpeedSetpoint(SpeedType type, float value, Vessel v)
        {
            this.type = type;
            this.value = value;
            this.v = v;
        }

        /// <summary>
        /// Convert to target SpeedType.
        /// WARNING: protentially time-consuming; accurate result NOT guaranteed
        /// </summary>
        /// <param name="t">The target SpeedType</param>
        /// <returns>Scalar value of the target speed type</returns>
        public SpeedSetpoint convert(SpeedType t)
        {
            if (t == type)
                return this;

            float mps = sameSystemMps();
            if (type.isSameSystem(t))
                return fromSameSystemMps(t, mps, v);
            else
            {
                // converting between systems
                if (type.isKinematicSystem())
                    return fromSameSystemMps(t, mps2EIAS(mps), v);
                else
                    return fromSameSystemMps(t, eias2Mps(mps), v);
            }
        }

        
        // unit convertions within the same system.

        /// <summary>
        /// Get speed in m/s of the current airspeed reference system.
        /// If converting between systems, use convert() instead.
        /// </summary>
        /// <returns>Speed in m/s</returns>
        public float sameSystemMps()
        {
            switch (type)
            {
                case SpeedType.MetersPerSecond:
                case SpeedType.IAS:
                    return value;
                case SpeedType.Knots:
                case SpeedType.KIAS:
                    return value * kts2mps;
                case SpeedType.Mach:
                    return value * (float)v.speedOfSound;
                default:
                    throw new NotImplementedException();
            }
        }

        private static SpeedSetpoint fromSameSystemMps(SpeedType t, float mps, Vessel v)
        {
            switch (t)
            {
                case SpeedType.MetersPerSecond:
                case SpeedType.IAS:
                    return new SpeedSetpoint(t, mps, v);
                case SpeedType.Knots:
                case SpeedType.KIAS:
                    return new SpeedSetpoint(t, mps * mps2kts, v);
                case SpeedType.Mach:
                    float m = mps / (float)v.speedOfSound;
                    if (float.IsNaN(m) || float.IsInfinity(m))
                        return new SpeedSetpoint(t, 0.0f, v);
                    else
                        return new SpeedSetpoint(t, m, v);
                default:
                    throw new NotImplementedException();
            }
        }

        // converting between the kinematic system and the pressure system
        // mps <=> IAS if FAR methods are available 
        // mps <=> EAS if otherwise
        private float mps2EIAS(float mps)
        {
            var RayleighPitotTubeStagPressureDelegate = AtmosphereAutopilot.Instance.farReflections.RayleighPitotTubeStagPressureDelegate;
            if (RayleighPitotTubeStagPressureDelegate != null)
                return mps2IAS(mps, RayleighPitotTubeStagPressureDelegate.Invoke);
            else
                return mps2EAS(mps);
        }

        private float eias2Mps(float eias)
        {
            var RayleighPitotTubeStagPressureDelegate = AtmosphereAutopilot.Instance.farReflections.RayleighPitotTubeStagPressureDelegate;
            if (RayleighPitotTubeStagPressureDelegate != null)
                return ias2Mps(eias, RayleighPitotTubeStagPressureDelegate.Invoke);
            else
                return eas2Mps(eias);
        }

        // IAS convertions if FAR methods are available
        private float mps2IAS(float mps, Func<double, double> rayleighPitotTubeStagPressure)
        {
            double M = mps / v.speedOfSound;

            if (double.IsNaN(M) || double.IsInfinity(M))
                return 0.0f;

            double presRatio = rayleighPitotTubeStagPressure(M);

            double velocity = presRatio - 1;
            velocity *= v.staticPressurekPa * 1000 * 2;
            velocity /= 1.225;
            velocity = Math.Sqrt(velocity);

            return (float)velocity;
        }
        private float ias2Mps(float ias, Func<double, double> rayleighPitotTubeStagPressure)
        {
            double presRatio = ias * ias * 1.225 / v.staticPressurekPa / 1000 / 2 + 1;

            if (double.IsNaN(presRatio) || presRatio <= 0.0 || presRatio >= 1e6)
                return 0.0f;

            var result = Common.Secant(
                rayleighPitotTubeStagPressure,
                presRatio,
                0.0,
                2.0,
                1e-7,
                50,
                0.0
                );

            if (result.Item1 != Common.AlgoStatus.Success)
            {
                var debugOutput = new { ias, presRatio, v.speedOfSound, v.mainBody.name, v.mainBody.atmosphereAdiabaticIndex };

                Debug.LogWarning("[AtmosphereAutopilot]: Secant method failed (" + result.Item1.ToString() + ") for ias2Mps; debug info " + debugOutput.ToString());
            }
            double M = result.Item2;

            return (float)(M * v.speedOfSound);
        }

        // Fall-back EAS convertions
        private float mps2EAS(float mps)
        {
            return Mathf.Sqrt(mps * mps * (float)v.atmDensity);
        }
        private float eas2Mps(float eas)
        {
            float mps = Mathf.Sqrt(eas * eas / (float)v.atmDensity);
            if (float.IsNaN(mps) || float.IsInfinity(mps))
                return 0.0f;
            return mps;
        }
    }

    /// <summary>
    /// Naive thrust controller for regular plane flight.
    /// </summary>
    public sealed class ProgradeThrustController : SubStateController
    {
        FlightModel imodel;

        FARReflections farReflections;

        internal ProgradeThrustController(Vessel v)
            : base(v, "Prograde thrust controller", 88437224)
        {
            // FAR IAS support
            farReflections = AtmosphereAutopilot.Instance.farReflections;

            pid.IntegralClamp = double.PositiveInfinity;
            pid.AccumulDerivClamp = double.PositiveInfinity;
            pid.KP = 0.4;
            pid.KI = 0.1;
            pid.KD = 0.5;
            setpoint = new SpeedSetpoint(SpeedType.MetersPerSecond, 100.0f, v);
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
        public float desired_v = 0.0f;

        [AutoGuiAttr("current_v", false, "G5")]
        public double current_v = 0.0f;

        [VesselSerializable("break_margin")]
        [AutoGuiAttr("break spd margin %", true, "G5")]
        public double break_margin = 10.0f;

        [GlobalSerializable("use_breaks")]
        [AutoGuiAttr("Use breaks", true)]
        public bool use_breaks = true;

        [AutoGuiAttr("prograde_thrust", false, "G5")]
        public double prograde_thrust;

        //[AutoGuiAttr("drag_estimate", false, "G5")]
        // drag acceleration
        public double drag_estimate;

        [VesselSerializable("Kp_v_factor")]
        [AutoGuiAttr("Kp_v", true, "G5")]
        public double Kp_v = 0.5;

        [AutoGuiAttr("acc_filter_k", true, "G5")]
        public double acc_filter_k = 1.0;

        [AutoGuiAttr("relaxation_acc_error", true, "G5")]
        public double relaxation_acc_error = 0.1;

        public Vector3d surfspd_dir;

        private bool was_breaking_previously = true;

        /// <summary>
        /// Get current surface speed in m/s
        /// </summary>
        /// <returns>Surface speed in m/s</returns>
        public double currentSurfaceSpeed()
        {
            return imodel.surface_v_magnitude;
        }

        /// <summary>
        /// Get current IAS speed in m/s
        /// </summary>
        /// <returns>IAS in m/s</returns>
        public double currentIAS()
        {
            var activeVesselIASDelegate = farReflections.ActiveVesselIASDelegate;
            if (activeVesselIASDelegate != null)
                return activeVesselIASDelegate();
            else
                return vessel.indicatedAirSpeed;
        }

        /// <summary>
        /// Get current speed in m/s that is the same to the setpoint airspeed reference system
        /// </summary>
        /// <returns>Speed in m/s</returns>
        public double currentSpeedOfSameSystem()
        {
            if (setpoint.type.isKinematicSystem())
                return currentSurfaceSpeed();
            else
                return currentIAS();
        }

        /// <summary>
        /// Get current speed in m/s that is the same to the setpoint airspeed reference system
        /// </summary>
        /// <param name="magnitude">Ratio of kinematic speed to current speed system </param>
        /// <returns>Speed in m/s</returns>
        public double currentSpeedOfSameSystem(out double magnitude)
        {
            if (setpoint.type.isKinematicSystem())
            {
                magnitude = 1.0;
                return currentSurfaceSpeed();
            }
            else
            {
                double ss = currentSurfaceSpeed();
                double ias = currentIAS();
                magnitude = ss / ias;
                return ias;
            }
        }

        /// <summary>
        /// Get speed difference in m/s to the setpoint airspeed
        /// </summary>
        /// <returns>Speed difference in m/s</returns>
        public double speedDiff()
        {
            return currentSpeedOfSameSystem() - setpoint.sameSystemMps();
        }

        /// <summary>
        /// Get speed difference in m/s to the setpoint airspeed
        /// </summary>
        /// <param name="magnitude">Ratio of kinematic speed to current speed system </param>
        /// <returns>Speed difference in m/s</returns>
        public double speedDiff(out double magnitude)
        {
            if (setpoint.type.isKinematicSystem())
            {
                magnitude = 1.0;
                return currentSurfaceSpeed() - setpoint.sameSystemMps();
            }
            else
            {
                double ss = currentSurfaceSpeed();
                double ias = currentIAS();
                magnitude = ss / ias;
                return ias - setpoint.sameSystemMps();
            }
        }

        /// <summary>
        /// Main control function
        /// </summary>
        /// <param name="cntrl">Control state to change</param>
        public override void ApplyControl(FlightCtrlState cntrl)
        {
            double presAccMagnitude;
            current_v = currentSpeedOfSameSystem(out presAccMagnitude);

            desired_v = setpoint.sameSystemMps();

            // apply breaks if needed
            if (use_breaks)
            {
                if (vessel.LandedOrSplashed())
                {
                    // we're on ground
                    if (current_v > desired_v)
                    {
                        vessel.ActionGroups.SetGroup(KSPActionGroup.Brakes, true);
                        was_breaking_previously = true;
                    }
                    else
                    {
                        if (was_breaking_previously)
                            vessel.ActionGroups.SetGroup(KSPActionGroup.Brakes, false);
                        was_breaking_previously = false;
                    }
                }
                else
                {
                    // we're in flight
                    if (current_v > (1.0 + break_margin / 100.0) * desired_v)
                    {
                        vessel.ActionGroups.SetGroup(KSPActionGroup.Brakes, true);
                        was_breaking_previously = true;
                    }
                    else
                    {
                        if (was_breaking_previously)
                            vessel.ActionGroups.SetGroup(KSPActionGroup.Brakes, false);
                        was_breaking_previously = false;
                    }
                }
            }
            else
                was_breaking_previously = true;

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
                surfspd_dir = imodel.prev_surface_v.normalized;
                Vector3 thrust = imodel.prev_cntrl2world * imodel.engines_thrust_principal;
                prograde_thrust = Vector3.Dot(thrust, surfspd_dir);

                double current_acc = Vector3.Dot(imodel.sum_acc, surfspd_dir);
                drag_estimate = current_acc - prograde_thrust / imodel.sum_mass - Vector3d.Dot(imodel.gravity_acc + imodel.noninert_acc, surfspd_dir);

                v_error = desired_v - current_v;
                desired_acc = Kp_v * v_error * presAccMagnitude;
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
                surfspd_dir = imodel.surface_v.normalized;
                cntrl.mainThrottle = solve_thrust_req(prograde_thrust + thrust_error, prev_input);

                //prev_thrust = prograde_thrust;
            }

            prev_input = cntrl.mainThrottle;
        }

        /// <summary>
        /// Supplemental control function allows direct control over the throttle
        /// </summary>
        /// <param name="cntrl">Control state to change</param>
        /// <param name="throttle">Set throttle value (0.0f ~ 1.0f)</param>
        public void ApplyThrottle(FlightCtrlState cntrl, float throttle)
        {
            if (use_breaks && was_breaking_previously)
            {
                vessel.ActionGroups.SetGroup(KSPActionGroup.Brakes, false);
                was_breaking_previously = false;
            }

            cntrl.mainThrottle = Mathf.Clamp01(throttle);
        }

        //double prev_thrust;
        float prev_input;

        int[] throttle_directions;

        public double estimated_max_thrust = 0.0;

        float solve_thrust_req(double required_thrust, float prev_throttle)
        {
            if (imodel.engines.Count == 0)
                return 0.1f;

            Common.Realloc(ref throttle_directions, imodel.engines.Count);

            double predicted_thrust = 0.0;
            estimated_max_thrust = 0.0;
            for (int i = 0; i < imodel.engines.Count; i++)
            {
                //ModuleEngines eng = imodel.engines[i].engine;
                //double e_prograde_thrust = Vector3.Dot(imodel.engines[i].thrust, surfspd_dir);
                estimated_max_thrust += imodel.engines[i].estimated_max_thrust;
            }

            if (estimated_max_thrust <= 0.0)
                return 0.1f;

            bool spool_dir_changed = false;
            double desired_throttle = 0.0;
            iter_count = 0;
            do
            {
                predicted_thrust = 0.0;
                for (int i = 0; i < imodel.engines.Count; i++)
                {
                    ModuleEngines eng = imodel.engines[i].engine;
                    FlightModel.EngineMoment em = imodel.engines[i];
                    double e_throttle = eng.currentThrottle / eng.thrustPercentage * 100.0;
                    if (eng.useEngineResponseTime)
                    {
                        throttle_directions[i] = prev_throttle >= eng.currentThrottle ? 1 : -1;
                        if (Mathf.Abs(eng.currentThrottle - prev_throttle * eng.thrustPercentage * 0.01f) <= 1e-4f)
                        {
                            throttle_directions[i] = 0;
                            predicted_thrust += em.estimated_max_thrust * prev_throttle;
                        }
                        else
                        {
                            float spd = throttle_directions[i] > 0 ? eng.engineAccelerationSpeed : eng.engineDecelerationSpeed;
                            double predict_throttle = Common.lerp(e_throttle, prev_throttle, TimeWarp.fixedDeltaTime * spd);
                            predicted_thrust += em.estimated_max_thrust * predict_throttle;
                        }
                    }
                    else
                        predicted_thrust += em.estimated_max_thrust * prev_throttle;
                }

                double t_error = required_thrust - predicted_thrust;
                double thrust_authority = 0.0;
                for (int i = 0; i < imodel.engines.Count; i++)
                {
                    ModuleEngines eng = imodel.engines[i].engine;
                    FlightModel.EngineMoment em = imodel.engines[i];
                    if (eng.useEngineResponseTime && throttle_directions[i] != 0)
                    {
                        float spd = throttle_directions[i] > 0 ? eng.engineAccelerationSpeed : eng.engineDecelerationSpeed;
                        thrust_authority += TimeWarp.fixedDeltaTime * spd * em.estimated_max_thrust;
                    }
                    else
                        thrust_authority += em.estimated_max_thrust;
                }
                if (thrust_authority == 0.0)
                    return 0.1f;
                desired_throttle = Common.Clamp(prev_throttle + t_error / thrust_authority, 0.0, 1.0);

                // now check if we changed spooling direction
                spool_dir_changed = false;
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
                            //throttle_directions[i] = new_throttle_direction;
                        }
                    }
                }

                prev_throttle = (float)desired_throttle;

                iter_count++;
            } while (spool_dir_changed && iter_count <= imodel.engines.Count + 2);

            if (double.IsNaN(desired_throttle) || double.IsInfinity(desired_throttle))
                desired_throttle = 0.5;

            return (float)Common.Clamp(desired_throttle, 0.0, 1.0);
        }

        [AutoGuiAttr("iter_count", false)]
        public int iter_count;

        [AutoGuiAttr("v_error", false, "G5")]
        double v_error;

        [AutoGuiAttr("desired_acc", false, "G5")]
        double desired_acc;

        [AutoGuiAttr("acc_error", false, "G5")]
        double acc_error;

        [AutoGuiAttr("thrust_error", false, "G5")]
        double thrust_error;

        [GlobalSerializable("use_pid")]
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

        #region GUI

        string[] spd_str_arr = new string[] { "m/s", "kts", "Mach", "IAS", "kIAS" };
        string SpdStr(SpeedType t)
        {
            return spd_str_arr[(int)t];
        }

        public bool spd_control_enabled = false;

        [VesselSerializable("spd_type")]
        public SpeedType type = SpeedType.MetersPerSecond;

        [VesselSerializable("setpoint_field")]
        internal DelayedFieldFloat setpoint_field = new DelayedFieldFloat(100.0f, "G4");

        /// <summary>
        /// Current speed setpoint, wich is maintained by controller
        /// </summary>
        public SpeedSetpoint setpoint;

        bool need_to_show_change = false;
        float setpoint_change_counter = 0.0f;

        [AutoGuiAttr("hotkey_speed_factor", true, "G4")]
        [GlobalSerializable("hotkey_speed_factor")]
        public static float hotkey_speed_factor = 0.7f;

        [AutoGuiAttr("use_throttle_hotkeys", true)]
        [GlobalSerializable("use_throttle_hotkeys")]
        public static bool use_throttle_hotkeys = true;

        [AutoGuiAttr("disable_on_thr_full_cut", true)]
        [GlobalSerializable("disable_on_thr_full_cut")]
        public static bool disable_on_thr_full_cut = false;

        [GlobalSerializable("spd_control_toggle_key")]
        [AutoHotkeyAttr("Speed control toggle")]
        static KeyCode spd_control_toggle_key = KeyCode.None;

        public override void OnUpdate()
        {
            //bool changed = false;
            if (Input.GetKeyDown(spd_control_toggle_key))
            {
                spd_control_enabled = !spd_control_enabled;
                MessageManager.post_status_message(spd_control_enabled ? "Speed control enabled" : "Speed control disabled");
                //changed = true;
            }

            if (disable_on_thr_full_cut && spd_control_enabled &&
                !FlightDriver.Pause && InputLockManager.IsUnlocked(ControlTypes.THROTTLE))
            {
                if (GameSettings.THROTTLE_FULL.GetKey() || 
                    GameSettings.THROTTLE_CUTOFF.GetKey())
                {
                    // turn speed control off if throttle full/cutoff key is pressed
                    spd_control_enabled = false;
                    MessageManager.post_status_message("Speed control disabled");
                }
            }

            if (use_throttle_hotkeys && spd_control_enabled &&
                !FlightDriver.Pause && InputLockManager.IsUnlocked(ControlTypes.THROTTLE))
            {
                // let's handle hotkey speed changing
                bool changed_by_hotkey = false;
                float new_vs = 0.0f;
                if (GameSettings.THROTTLE_UP.GetKey() && !GameSettings.MODIFIER_KEY.GetKey())
                {
                    float ms = setpoint.value;
                    new_vs = ms + Time.deltaTime * hotkey_speed_factor * ms;
                    changed_by_hotkey = true;
                }
                else if (GameSettings.THROTTLE_DOWN.GetKey() && !GameSettings.MODIFIER_KEY.GetKey())
                {
                    float ms = setpoint.value;
                    new_vs = ms - Time.deltaTime * hotkey_speed_factor * ms;
                    changed_by_hotkey = true;
                }

                if (changed_by_hotkey)
                {
                    setpoint.value = new_vs;
                    need_to_show_change = true;
                    setpoint_change_counter = 0;
                    setpoint_field.Value = new_vs;
                    //changed = true;
                }

                if (need_to_show_change)
                    setpoint_change_counter += Time.deltaTime;
                if (setpoint_change_counter > 1.0f)
                {
                    setpoint_change_counter = 0;
                    need_to_show_change = false;
                }
            }
            else
            {
                need_to_show_change = false;
                setpoint_change_counter = 0;
            }

            AtmosphereAutopilot.Instance.mainMenuGUISpeedUpdate();
        }

        protected override void OnGUICustomAlways()
        {
            if (need_to_show_change)
            {
                Rect rect = new Rect(Screen.width / 2.0f - 80.0f, 120.0f, 160.0f, 20.0f);
                string str = "SPD = " + setpoint.value.ToString("G4") + " " + SpdStr(setpoint.type);
                GUI.Label(rect, str, GUIStyles.hoverLabel);
            }

            setpoint_field.OnUpdate();      // handle delay
        }

        /// <summary>
        /// Standard speed control GUI block to integrate in other controllers
        /// </summary>
        /// <returns>true if speed control is enabled</returns>
        public bool SpeedCtrlGUIBlock()
        {
            spd_control_enabled = GUILayout.Toggle(spd_control_enabled, "Speed control", GUIStyles.toggleButtonStyle);
            GUILayout.BeginHorizontal();

            SpeedType oldType = type;
            for (int i = 0; i < 5; i++)
            {
                if (GUILayout.Toggle(type == (SpeedType)i, spd_str_arr[i], GUIStyles.toggleButtonStyle))
                {
                    type = (SpeedType)i;
                }
            }
            GUILayout.EndHorizontal();

            setpoint_field.DisplayLayout(GUIStyles.textBoxStyle);

            if (oldType != type)
            {
                // need to convert old setpoint to new format
                setpoint = setpoint.convert(type);
                setpoint_field.Value = setpoint.value;
            }
            else
                setpoint = new SpeedSetpoint(type, setpoint_field, vessel);

            return spd_control_enabled;
        }

        #endregion
    }
}
