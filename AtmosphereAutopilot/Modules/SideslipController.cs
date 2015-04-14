using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace AtmosphereAutopilot
{
    public sealed class SideslipController : SIMOController
    {
        InstantControlModel model;
        MediumFlightModel mmodel;
        YawAngularVelocityController v_controller;

        PController pid = new PController();

        internal SideslipController(Vessel v) :
            base(v, "Sideslip controller", 88437222) { AutoTrim = true; }

        public override void InitializeDependencies(Dictionary<Type, AutopilotModule> modules)
        {
            model = modules[typeof(InstantControlModel)] as InstantControlModel;
            mmodel = modules[typeof(MediumFlightModel)] as MediumFlightModel;
            v_controller = modules[typeof(YawAngularVelocityController)] as YawAngularVelocityController;
        }

        protected override void OnActivate()
        {
            model.Activate();
            mmodel.Activate();
            v_controller.Activate();
        }

        protected override void OnDeactivate()
        {
            model.Deactivate();
            mmodel.Deactivate();
            v_controller.Deactivate();
        }

        double time_in_regime = 0.0;

        /// <summary>
        /// Main control function
        /// </summary>
        /// <param name="cntrl">Control state to change</param>
        public override double ApplyControl(FlightCtrlState cntrl, double target_value)
        {
            const double degree_to_rad = Math.PI / 180.0;

            input = -mmodel.Sideslip;

            // Adapt KP, so that on max_angular_v it produces max_angular_dv * kp_acc factor output
            if (mmodel.MaxAngularSpeed(YAW) != 0.0)
            {
                pid.KP = kp_vel_factor * fbw_max_sideslip / mmodel.MaxAngularSpeed(YAW);
            }

            double user_input = ControlUtils.get_neutralized_user_input(cntrl, YAW);
            if (user_input != 0.0)
                desired_sideslip = user_input * fbw_max_sideslip * degree_to_rad;
            else
                desired_sideslip = target_value;
                
            desired_v = pid.Control(input, desired_sideslip);

            output = Common.Clamp(desired_v, mmodel.MaxAngularSpeed(YAW));

            error = desired_sideslip - input;

            ControlUtils.neutralize_user_input(cntrl, YAW);
            v_controller.ApplyControl(cntrl, output);

            // check if we're stable on given input value
            if (AutoTrim)
            {
                if (Math.Abs(input) < 5e-3)
                {
                    time_in_regime += TimeWarp.fixedDeltaTime;
                }
                else
                {
                    time_in_regime = 0.0;
                }

                if (time_in_regime >= 5.0)
                    ControlUtils.set_trim(YAW, model);
            }

            return output;
        }


        #region Parameters

        [VesselSerializable("kp_vel_factor")]
        [GlobalSerializable("kp_vel_factor")]
        [AutoGuiAttr("KP velocity factor", true, "G6")]
        double kp_vel_factor = 0.5;

        [GlobalSerializable("fbw_max_sideslip")]
        [VesselSerializable("fbw_max_sideslip")]
        [AutoGuiAttr("max Sideslip in degrees", true, "G6")]
        double fbw_max_sideslip = 10.0;

        [AutoGuiAttr("DEBUG desired_v", false, "G8")]
        double desired_v;

        [AutoGuiAttr("DEBUG desired_sideslip", false, "G8")]
        double desired_sideslip;

        [GlobalSerializable("AutoTrim")]
        [AutoGuiAttr("AutoTrim", true, null)]
        public bool AutoTrim { get; set; }

        #endregion
    }
}
