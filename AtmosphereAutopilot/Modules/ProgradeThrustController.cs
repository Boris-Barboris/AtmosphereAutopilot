/*
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

        public float convert(SpeedType t)
        {
            if (t == type)
                return value;
            float mpersec = mps();
            switch (t)
            {
                case SpeedType.MetersPerSecond:
                    return mpersec;
                case SpeedType.Mach:
                    return (float)(mpersec / v.speedOfSound);
                case SpeedType.Knots:
                    return mpersec * mps2kts;
                case SpeedType.IAS:
                    return Mathf.Sqrt(mpersec * mpersec * (float)v.atmDensity);
                case SpeedType.KIAS:
                    return Mathf.Sqrt(mpersec * mpersec * mps2kts * mps2kts * (float)v.atmDensity);
                default:
                    return 0.0f;
            }
        }

        public static float convert_from_mps(SpeedType t, float mps, Vessel v)
        {
            switch (t)
            {
                case SpeedType.MetersPerSecond:
                    return mps;
                case SpeedType.Mach:
                    return (float)(mps / v.speedOfSound);
                case SpeedType.Knots:
                    return mps * mps2kts;
                case SpeedType.IAS:
                    return Mathf.Sqrt(mps * mps * (float)v.atmDensity);
                case SpeedType.KIAS:
                    return Mathf.Sqrt(mps * mps * mps2kts * mps2kts * (float)v.atmDensity);
                default:
                    return 0.0f;
            }
        }

        public float mps()
        {
            if (type == SpeedType.MetersPerSecond)
                return value;
            switch (type)
            {
                case SpeedType.Mach:
                    return (float)(value * v.speedOfSound);
                case SpeedType.Knots:
                    return value * kts2mps;
                case SpeedType.IAS:
                    return Mathf.Sqrt(value * value / (float)v.atmDensity);
                case SpeedType.KIAS:
                    return Mathf.Sqrt(value * value * kts2mps * kts2mps / (float)v.atmDensity);
                default:
                    return 0.0f;
            }
        }
    }

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

        /// <summary>
        /// Main control function
        /// </summary>
        /// <param name="cntrl">Control state to change</param>
        /// <param name="target_value">Prograde surface speed setpoint</param>
        public override float ApplyControl(FlightCtrlState cntrl, float target_value)
        {
            current_v = imodel.surface_v_magnitude;
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
                surfspd_dir = imodel.prev_surface_v.normalized;
                Vector3 thrust = imodel.prev_cntrl2world * imodel.engines_thrust_principal;
                prograde_thrust = Vector3.Dot(thrust, surfspd_dir);

                double current_acc = Vector3.Dot(imodel.sum_acc, surfspd_dir);
                drag_estimate = current_acc - prograde_thrust / imodel.sum_mass - Vector3d.Dot(imodel.gravity_acc + imodel.noninert_acc, surfspd_dir);

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
                surfspd_dir = imodel.surface_v.normalized;
                cntrl.mainThrottle = solve_thrust_req(prograde_thrust + thrust_error, prev_input);

                //prev_thrust = prograde_thrust;
            }

            prev_input = cntrl.mainThrottle;
            return cntrl.mainThrottle;
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

        static readonly string[] spd_str_arr = new string[] { "OFF", "ms", "kts", "M", "ias", "kias" };

        public int chosen_spd_mode = 0;
        public bool spd_control_enabled = false;

        [VesselSerializable("spd_type")]
        SpeedType type = SpeedType.MetersPerSecond;

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
                string str = "SPD = " + setpoint.value.ToString("G4");
                switch (setpoint.type)
                {
                    case SpeedType.IAS:
                        str = str + " IAS";
                        break;
                    case SpeedType.KIAS:
                        str = str + " kIAS";
                        break;
                    case SpeedType.Knots:
                        str = str + " kts";
                        break;
                    case SpeedType.Mach:
                        str = str + " M";
                        break;
                    case SpeedType.MetersPerSecond:
                        str = str + " m/s";
                        break;
                }
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
            for (int i = 1; i < 6; i++)
            {
                if (GUILayout.Toggle(chosen_spd_mode == i, spd_str_arr[i], GUIStyles.toggleButtonStyle))
                    chosen_spd_mode = i;
            }
            GUILayout.EndHorizontal();

            SpeedType newtype = type;
            if (chosen_spd_mode != 0)
                newtype = (SpeedType)(chosen_spd_mode - 1);

            setpoint_field.DisplayLayout(GUIStyles.textBoxStyle);

            if (newtype != type)
            {
                // need to convert old setpoint to new format
                setpoint_field.Value = setpoint.convert(newtype);
                setpoint = new SpeedSetpoint(newtype, setpoint_field, vessel);
                type = newtype;
            }
            else
                setpoint = new SpeedSetpoint(type, setpoint_field, vessel);

            return spd_control_enabled;
        }

        #endregion
    }
}
