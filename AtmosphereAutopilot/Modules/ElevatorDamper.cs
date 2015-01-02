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
    class ElevatorDamper: Dampener
    {
        public ElevatorDamper(Vessel cur_vessel)
            : base(cur_vessel) 
        {
            pid = new PIDController();
            pid.KP = 1.0;
            pid.KI = 3.0;
            pid.IntegralClamp = 1.0;
            pid.KD = 0.001;
        }

        PIDController pid;
        double time = 0.0;
        double pitch_angular_velocity;
        double output;

        protected override void apply_module(FlightCtrlState cntrl)
        {
            // vector to right wing
            pitch_angular_velocity = -currentVessel.angularVelocity.x;
            time = time + TimeWarp.fixedDeltaTime;
            output = pid.Control(pitch_angular_velocity, 0.0, time);
            
            // check if user is inputing control
            if (cntrl.killRot)                          // when sas works just back off
                return;
            if (cntrl.pitch == cntrl.pitchTrim)         // when user doesn't use control, pitch is on the same level as trim
            {
                cntrl.pitch = (float)Common.Clamp(output, 1.0);
                cntrl.pitchTrim = cntrl.pitch;
            }
            else
                pid.clear();
            if (currentVessel.checkLanded())
                pid.clear();
        }

        bool gui_shown = false;
        public override void toggleGUI()
        {
            gui_shown = !gui_shown;
            if (gui_shown)
            {
                RenderingManager.AddToPostDrawQueue(5, drawGUI);
                kp_str = pid.KP.ToString("G8");
                ki_str = pid.KI.ToString("G8");
                kd_str = pid.KD.ToString("G8");
                ic_str = pid.IntegralClamp.ToString("G8");
            }
            else
                RenderingManager.RemoveFromPostDrawQueue(5, drawGUI);
        }

        Rect window = new Rect(50.0f, 100.0f, 300.0f, 250.0f);
        protected override void drawGUI()
        {
            if (!gui_shown)
                return;
            window = GUILayout.Window(9803294, window, _drawGUI, "Pitch damper", GUILayout.Height(0), GUILayout.MinWidth(233));
        }

        string kp_str = "";
        string ki_str = "";
        string kd_str = "";
        string ic_str = "";

        void _drawGUI(int id)
        {
            GUILayout.BeginVertical();
            GUILayout.Label("Angular vel vector = " + currentVessel.angularVelocity.ToString());
            GUILayout.Label("Angular velocity = " + pitch_angular_velocity.ToString("G8"));
            GUILayout.Label("Output = " + output.ToString("G8"));
            GUILayout.Label("Accumulator = " + pid.Accumulator.ToString("G8"));

            GUILayout.Label("KP = ");
            kp_str = GUILayout.TextField(kp_str);
            try
            {
                pid.KP = double.Parse(kp_str);
            }
            catch { }

            GUILayout.Label("KI = ");
            ki_str = GUILayout.TextField(ki_str);
            try
            {
                pid.KI = double.Parse(ki_str);
            }
            catch { }

            GUILayout.Label("KD = ");
            kd_str = GUILayout.TextField(kd_str);
            try
            {
                pid.KD = double.Parse(kd_str);
            }
            catch { }

            GUILayout.Label("Integral clamp = ");
            ic_str = GUILayout.TextField(ic_str);
            try
            {
                pid.IntegralClamp = double.Parse(ic_str);
            }
            catch { }

            GUILayout.EndVertical();
            GUI.DragWindow();
        }
    }
}
