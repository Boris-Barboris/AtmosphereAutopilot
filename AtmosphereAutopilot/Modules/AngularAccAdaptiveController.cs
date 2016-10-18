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
using System.IO;

namespace AtmosphereAutopilot
{
    /// <summary>
    /// Controls angular acceleration. Meant to be used by AngularVelAdaptiveController
    /// </summary>
    public abstract class AngularAccAdaptiveController : SISOController
    {
        protected int axis;

        protected FlightModel imodel;

        // Telemetry writers
        protected StreamWriter controlWriter, v_writer, acc_writer, prediction_writer,
            desire_acc_writer, aoa_writer, airspd_writer, density_writer, outputWriter;

        /// <summary>
        /// Create controller instance.
        /// </summary>
        /// <param name="vessel">Vessel to control</param>
        /// <param name="module_name">Name of controller</param>
        /// <param name="wnd_id">unique for types window id</param>
        /// <param name="axis">Pitch = 0, roll = 1, yaw = 2</param>
        protected AngularAccAdaptiveController(Vessel vessel, string module_name,
            int wnd_id, int axis)
            : base(vessel, module_name, wnd_id)
        {
            this.axis = axis;
        }

        public override void InitializeDependencies(Dictionary<Type, AutopilotModule> modules)
        {
            this.imodel = modules[typeof(FlightModel)] as FlightModel;
        }

        protected override void OnActivate()
        {
            imodel.Activate();
        }

        protected override void OnDeactivate()
        {
            write_telemetry = false;
            imodel.Deactivate();
        }

        [AutoGuiAttr("angular acc", false, "G6")]
        protected float acc;

        //[AutoGuiAttr("model acc", false, "G8")]
        //protected float model_acc;

        [AutoGuiAttr("output", false, "G6")]
        public float output;

        /// <summary>
        /// Main control function
        /// </summary>
        /// <param name="cntrl">Control state to change</param>
        /// <param name="target_value">Desired angular acceleration</param>
        public override float ApplyControl(FlightCtrlState cntrl, float target_value)
        {
            acc = (float)imodel.AngularAcc(axis);
            //model_acc = (float)imodel.model_acc[axis];
            desired_acc = target_value;

            if (write_telemetry)
            {
                desire_acc_writer.Write(target_value.ToString("G8") + ',');
                acc_writer.Write(acc.ToString("G8") + ',');
                v_writer.Write(imodel.AngularVel(axis).ToString("G8") + ',');
                //prediction_writer.Write(target_value.ToString("G8") + ',');
                aoa_writer.Write(imodel.AoA(axis).ToString("G8") + ',');
                airspd_writer.Write((imodel.up_srf_v + imodel.fwd_srf_v).magnitude.ToString("G8") + ',');
                density_writer.Write(vessel.atmDensity.ToString("G8") + ',');
            }

            float cur_input_raw = get_required_input(cntrl, desired_acc);
            output = cur_input_raw;
            if (float.IsNaN(output) || float.IsInfinity(output))
                output = 0.0f;

            // fighting numerical precision issues
            //if (axis == ROLL && imodel.dyn_pressure > 1000.0 && Mathf.Abs(output) < 0.006f)
            //    output = 0.0f;

            ControlUtils.set_raw_output(cntrl, axis, output);

            if (write_telemetry)
            {
                controlWriter.Write(csurf_output.ToString("G8") + ',');
                outputWriter.Write(output.ToString("G8") + ',');
            }

            return output;
        }

        protected virtual float get_required_input(FlightCtrlState cntrl, float target_value)
        {
            return ControlUtils.getControlFromState(cntrl, axis);
        }

        [AutoGuiAttr("Csurf output", false, "G6")]
        protected float csurf_output { get { return imodel.ControlSurfPos(axis); } }

        [AutoGuiAttr("Write telemetry", true)]
        protected bool write_telemetry
        {
            get { return _write_telemetry; }
            set
            {
                if (value)
                {
                    if (!_write_telemetry)
                    {
                        create_writers();
                        _write_telemetry = value;
                    }
                }
                else
                {
                    if (_write_telemetry)
                    {
                        close_writers();
                        _write_telemetry = value;
                    }
                }
            }
        }
        bool _write_telemetry = false;

        [AutoGuiAttr("desired acc", false, "G6")]
        internal float desired_acc { get; private set; }

        void create_writers()
        {
            controlWriter = File.CreateText(KSPUtil.ApplicationRootPath + "/Resources/control.csv");
            v_writer = File.CreateText(KSPUtil.ApplicationRootPath + "/Resources/v.csv");
            acc_writer = File.CreateText(KSPUtil.ApplicationRootPath + "/Resources/acc.csv");
            desire_acc_writer = File.CreateText(KSPUtil.ApplicationRootPath + "/Resources/desire.csv");
            prediction_writer = File.CreateText(KSPUtil.ApplicationRootPath + "/Resources/predict.csv");
            aoa_writer = File.CreateText(KSPUtil.ApplicationRootPath + "/Resources/aoa.csv");
            airspd_writer = File.CreateText(KSPUtil.ApplicationRootPath + "/Resources/airspd.csv");
            density_writer = File.CreateText(KSPUtil.ApplicationRootPath + "/Resources/density.csv");
            outputWriter = File.CreateText(KSPUtil.ApplicationRootPath + "/Resources/output.csv");
        }

        void close_writers()
        {
            controlWriter.Close();
            v_writer.Close();
            acc_writer.Close();
            desire_acc_writer.Close();
            prediction_writer.Close();
            aoa_writer.Close();
            airspd_writer.Close();
            density_writer.Close();
            outputWriter.Close();
        }

    }


    //
    // Three realizations
    //

    public abstract class PitchYawAngularAccController : AngularAccAdaptiveController
    {
        protected PitchYawAngularAccController(Vessel vessel, string module_name,
            int wnd_id, int axis)
            : base(vessel, module_name, wnd_id, axis)
        { }

        protected Matrix cur_state = new Matrix(4, 1);
        protected Matrix input_mat = new Matrix(1, 1);

        [AutoGuiAttr("model_predicted_acc", false, "G6")]
        protected double model_predicted_acc;

        protected double cur_model_acc, prev_model_acc;

        [AutoGuiAttr("authority", false, "G6")]
        protected double authority;

        protected LinearSystemModel lin_model, lin_model_undelayed;

        protected override float get_required_input(FlightCtrlState cntrl, float target_value)
        {
            float new_input = 0.0f;

            if (AtmosphereAutopilot.AeroModel == AtmosphereAutopilot.AerodinamycsModel.FAR)
            {
                authority = lin_model.B[1, 0];

                // check if we have inadequate model authority
                if (authority < 1e-4)
                {
                    float user_input = axis == PITCH ? cntrl.pitch : cntrl.yaw;
                    if (user_input != 0.0f)
                        return user_input;
                    else
                        return Common.Clampf(target_value, 1.0f);
                }

                // get model prediction for next frame
                cur_state[0, 0] = imodel.AoA(axis);
                cur_state[1, 0] = imodel.AngularVel(axis);
                cur_state[2, 0] = imodel.ControlSurfPos(axis);
                cur_state[3, 0] = imodel.GimbalPos(axis);
                input_mat[0, 0] = cur_state[2, 0];
                double cur_acc_prediction = lin_model.eval_row(1, cur_state, input_mat);

                double acc_error = target_value - cur_acc_prediction;
                new_input = (float)(imodel.ControlSurfPos(axis) + acc_error / authority);
                new_input = Common.Clampf(new_input, 1.0f);

                // Exponential blend can mess with rotation model, let's check it
                if (FlightModel.far_blend_collapse(imodel.ControlSurfPos(axis), new_input))
                {
                    // we need to recalculate new_input according to undelayed model
                    authority = lin_model_undelayed.B[1, 0];
                    new_input = (float)(imodel.ControlSurfPos(axis) + acc_error / authority);
                    new_input = Common.Clampf(new_input, 1.0f);
                }

                model_predicted_acc = cur_acc_prediction + authority * (new_input - cur_state[2, 0]);
            }
            else
            {
                authority = lin_model_undelayed.B[1, 0];

                // check if we have inadequate model authority
                if (authority < 1e-4)
                {
                    float user_input = axis == PITCH ? cntrl.pitch : cntrl.yaw;
                    if (user_input != 0.0f)
                        return user_input;
                    else
                        return Common.Clampf(target_value, 1.0f);
                }

                // get model prediction for next frame
                cur_state[0, 0] = imodel.AoA(axis);
                cur_state[1, 0] = imodel.AngularVel(axis);
                cur_state[2, 0] = imodel.GimbalPos(axis);
                input_mat[0, 0] = imodel.ControlSurfPos(axis);
                double cur_acc_prediction = lin_model_undelayed.eval_row(1, cur_state, input_mat);

                double acc_error = target_value - cur_acc_prediction;
                new_input = (float)(input_mat[0, 0] + acc_error / authority);
                new_input = Common.Clampf(new_input, 1.0f);

                if (Math.Abs(new_input - input_mat[0, 0]) / TimeWarp.fixedDeltaTime > SyncModuleControlSurface.CSURF_SPD)
                {
                    // we're exceeding control surface speed
                    cur_state[3, 0] = cur_state[2, 0];
                    cur_state[2, 0] = input_mat[0, 0];
                    if (new_input > input_mat[0, 0])
                        cur_state[2, 0] += TimeWarp.fixedDeltaTime * SyncModuleControlSurface.CSURF_SPD;
                    else
                        cur_state[2, 0] -= TimeWarp.fixedDeltaTime * SyncModuleControlSurface.CSURF_SPD;
                    cur_state[2, 0] = Common.Clamp(cur_state[2, 0], 1.0);
                    input_mat[0, 0] = cur_state[2, 0];
                    cur_acc_prediction = lin_model.eval_row(1, cur_state, input_mat);
                    acc_error = target_value - cur_acc_prediction;
                    authority = lin_model.B[1, 0];
                    new_input = (float)(cur_state[2, 0] + acc_error / authority);
                    new_input = Common.Clampf(new_input, 1.0f);
                }
                model_predicted_acc = cur_acc_prediction + authority * (new_input - input_mat[0, 0]);
            }

            if (write_telemetry)
            {
                prediction_writer.Write(model_predicted_acc.ToString("G8") + ',');
            }

            return new_input;
        }
    }

    public sealed class PitchAngularAccController : PitchYawAngularAccController
    {
        internal PitchAngularAccController(Vessel vessel)
            : base(vessel, "Pitch ang acc controller", 77821329, PITCH)
        { }

        public override void InitializeDependencies(Dictionary<Type, AutopilotModule> modules)
        {
            base.InitializeDependencies(modules);
            lin_model = imodel.pitch_rot_model;
            lin_model_undelayed = imodel.pitch_rot_model_undelayed;
        }
    }

    public sealed class YawAngularAccController : PitchYawAngularAccController
    {
        internal YawAngularAccController(Vessel vessel)
            : base(vessel, "Yaw ang acc controller", 77821331, YAW)
        { }

        public override void InitializeDependencies(Dictionary<Type, AutopilotModule> modules)
        {
            base.InitializeDependencies(modules);
            lin_model = imodel.yaw_rot_model;
            lin_model_undelayed = imodel.yaw_rot_model_undelayed;
        }
    }

    public sealed class RollAngularAccController : AngularAccAdaptiveController
    {
        internal RollAngularAccController(Vessel vessel)
            : base(vessel, "Roll ang acc controller", 77821330, ROLL)
        { }

        AngularAccAdaptiveController yc;

        public override void InitializeDependencies(Dictionary<Type, AutopilotModule> modules)
        {
            base.InitializeDependencies(modules);
            yc = modules[typeof(YawAngularAccController)] as AngularAccAdaptiveController;
        }

        Matrix cur_state = new Matrix(3, 1);
        Matrix input_mat = new Matrix(3, 1);

        [AutoGuiAttr("model_predicted_acc", false, "G6")]
        double model_predicted_acc;

        //double cur_model_acc;
        //double prev_model_acc;

        [AutoGuiAttr("authority", false, "G6")]
        double authority;

        protected override float get_required_input(FlightCtrlState cntrl, float target_value)
        {
            float new_input = 0.0f;

            if (AtmosphereAutopilot.AeroModel == AtmosphereAutopilot.AerodinamycsModel.FAR)
            {
                authority = imodel.roll_rot_model.B[0, 0];
                // check if we have inadequate model authority
                if (authority < 1e-4)
                {
                    float user_input = cntrl.roll;
                    if (user_input != 0.0f)
                        return user_input;
                    else
                        return Common.Clampf(target_value, 1.0f);
                }

                // get model prediction for next frame
                cur_state[0, 0] = imodel.AngularVel(ROLL);
                cur_state[1, 0] = imodel.ControlSurfPos(ROLL);
                cur_state[2, 0] = imodel.GimbalPos(ROLL);
                input_mat[0, 0] = cur_state[1, 0];
                input_mat[1, 0] = FlightModel.far_exponential_blend(imodel.ControlSurfPos(YAW), yc.output);
                input_mat[2, 0] = imodel.AoA(YAW);
                double cur_acc_prediction = imodel.roll_rot_model.eval_row(0, cur_state, input_mat);

                double acc_error = target_value - cur_acc_prediction;
                new_input = (float)(imodel.ControlSurfPos(ROLL) + acc_error / authority);
                new_input = Common.Clampf(new_input, 1.0f);

                // Exponential blend can mess with rotation model, let's check it
                if (FlightModel.far_blend_collapse(imodel.ControlSurfPos(ROLL), new_input))
                {
                    // we need to recalculate new_input according to undelayed model
                    authority = imodel.roll_rot_model_undelayed.B[0, 0];
                    new_input = (float)(imodel.ControlSurfPos(ROLL) + acc_error / authority);
                    new_input = Common.Clampf(new_input, 1.0f);
                }

                model_predicted_acc = cur_acc_prediction + authority * (new_input - cur_state[1, 0]);
            }
            else
            {
                authority = imodel.roll_rot_model_undelayed.B[0, 0];
                // check if we have inadequate model authority
                if (authority < 1e-4)
                {
                    float user_input = cntrl.roll;
                    if (user_input != 0.0f)
                        return user_input;
                    else
                        return Common.Clampf(target_value, 1.0f);
                }

                // get model prediction for next frame
                cur_state[0, 0] = imodel.AngularVel(ROLL);
                cur_state[1, 0] = imodel.GimbalPos(ROLL);
                input_mat[0, 0] = imodel.ControlSurfPos(ROLL);
                input_mat[1, 0] = FlightModel.stock_actuator_blend(imodel.ControlSurfPos(YAW), yc.output);
                input_mat[2, 0] = imodel.AoA(YAW);
                double cur_acc_prediction = imodel.roll_rot_model_undelayed.eval_row(0, cur_state, input_mat);

                double acc_error = target_value - cur_acc_prediction;
                new_input = (float)(input_mat[0, 0] + acc_error / authority);
                new_input = Common.Clampf(new_input, 1.0f);

                if (Math.Abs(new_input - input_mat[0, 0]) / TimeWarp.fixedDeltaTime > SyncModuleControlSurface.CSURF_SPD)
                {
                    // we're exceeding control surface speed
                    cur_state[2, 0] = cur_state[1, 0];
                    cur_state[1, 0] = input_mat[0, 0];
                    if (new_input > input_mat[0, 0])
                        cur_state[1, 0] += TimeWarp.fixedDeltaTime * SyncModuleControlSurface.CSURF_SPD;
                    else
                        cur_state[1, 0] -= TimeWarp.fixedDeltaTime * SyncModuleControlSurface.CSURF_SPD;
                    cur_state[1, 0] = Common.Clamp(cur_state[1, 0], 1.0);
                    input_mat[0, 0] = cur_state[1, 0];
                    cur_acc_prediction = imodel.roll_rot_model.eval_row(0, cur_state, input_mat);
                    acc_error = target_value - cur_acc_prediction;
                    authority = imodel.roll_rot_model.B[0, 0];
                    new_input = (float)(cur_state[1, 0] + acc_error / authority);
                    new_input = Common.Clampf(new_input, 1.0f);
                }
                model_predicted_acc = cur_acc_prediction + authority * (new_input - input_mat[0, 0]);
            }

            if (write_telemetry)
            {
                prediction_writer.Write(model_predicted_acc.ToString("G8") + ',');
            }

            return new_input;
        }
    }

}
