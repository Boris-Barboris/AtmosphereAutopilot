using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;

namespace AtmosphereAutopilot
{
    /// <summary>
    /// Any autopilot component. For example, roll damper
    /// </summary>
    abstract class PIDAngularVelDampener
    {
        protected Vessel currentVessel = null;
        protected bool enabled = false;
        public bool Enabled { get { return enabled; } }

        public PIDAngularVelDampener(Vessel cur_vessel, string damper_name, int wnd_id)
        {
            currentVessel = cur_vessel;
            this.damper_name = damper_name;
            this.wnd_id = wnd_id;
        }

        protected PIDController pid;
        protected double angular_velocity;
        protected double output;

        public void Activate()
        {
            currentVessel.OnAutopilotUpdate += new FlightInputCallback(OnFixedUpdate);
            enabled = true;
        }

        public void Deactivate()
        {
            currentVessel.OnAutopilotUpdate -= new FlightInputCallback(OnFixedUpdate);
            enabled = false;
        }

        protected bool gui_shown = false;
        public void toggleGUI()
        {
            gui_shown = !gui_shown;
            if (gui_shown)
            {
                RenderingManager.AddToPostDrawQueue(5, drawGUI);
                kp_str = pid.KP.ToString("G8");
                ki_str = pid.KI.ToString("G8");
                kd_str = pid.KD.ToString("G8");
                ic_str = pid.IntegralClamp.ToString("G8");
                ac_str = pid.AccumulatorClamp.ToString("G8");
                adc_str = pid.AccumulDerivClamp.ToString("G8");
            }
            else
                RenderingManager.RemoveFromPostDrawQueue(5, drawGUI);
        }

        protected string damper_name;
        protected int wnd_id;
        protected Rect window = new Rect(50.0f, 100.0f, 250.0f, 250.0f);

        void drawGUI()
        {
            if (!gui_shown)
                return;
            window = GUILayout.Window(wnd_id, window, _drawGUI, damper_name, GUILayout.Height(0), GUILayout.MinWidth(200));
        }

        string kp_str = "";
        string ki_str = "";
        string kd_str = "";
        string ic_str = "";
        string ac_str = "";
        string adc_str = "";

        void _drawGUI(int id)
        {
            GUILayout.BeginVertical();
            GUILayout.Label("Angular velocity = " + angular_velocity.ToString("G8"));
            GUILayout.Label("Output = " + output.ToString("G8"));
            GUILayout.Label("Accumulator = " + pid.Accumulator.ToString("G8"));

            GUILayout.BeginHorizontal();
            GUILayout.Label("KP = ");
            kp_str = GUILayout.TextField(kp_str);
            GUILayout.EndHorizontal();
            try
            {
                pid.KP = double.Parse(kp_str);
            }
            catch { }

            GUILayout.BeginHorizontal();
            GUILayout.Label("KI = ");
            ki_str = GUILayout.TextField(ki_str);
            GUILayout.EndHorizontal();
            try
            {
                pid.KI = double.Parse(ki_str);
            }
            catch { }

            GUILayout.BeginHorizontal();
            GUILayout.Label("KD = ");
            kd_str = GUILayout.TextField(kd_str);
            GUILayout.EndHorizontal();
            try
            {
                pid.KD = double.Parse(kd_str);
            }
            catch { }

            GUILayout.BeginHorizontal();
            GUILayout.Label("IntegralClamp = ");
            ic_str = GUILayout.TextField(ic_str);
            GUILayout.EndHorizontal();
            try
            {
                pid.IntegralClamp = double.Parse(ic_str);
            }
            catch { }

            GUILayout.BeginHorizontal();
            GUILayout.Label("AccumulatorClamp = ");
            ac_str = GUILayout.TextField(ac_str);
            GUILayout.EndHorizontal();
            try
            {
                pid.AccumulatorClamp = double.Parse(ac_str);
            }
            catch { }

            GUILayout.BeginHorizontal();
            GUILayout.Label("AccumulDerivClamp = ");
            adc_str = GUILayout.TextField(adc_str);
            GUILayout.EndHorizontal();
            try
            {
                pid.AccumulDerivClamp = double.Parse(adc_str);
            }
            catch { }

            GUILayout.EndVertical();
            GUI.DragWindow();
        }

        protected abstract void OnFixedUpdate(FlightCtrlState cntrl);
    }
}
