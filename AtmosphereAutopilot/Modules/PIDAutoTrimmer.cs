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
    abstract class PIDAutoTrimmer
    {
        protected Vessel currentVessel = null;
        protected bool enabled = false;
        public bool Enabled { get { return enabled; } }

        public PIDAutoTrimmer(Vessel cur_vessel, string damper_name, int wnd_id)
        {
            currentVessel = cur_vessel;
            this.damper_name = damper_name;
            this.wnd_id = wnd_id;
            pid = new PIDController();
        }

        protected virtual bool loadFromPreset(string NodeName)
        {
            loadGlobalData(NodeName);
            ConfigNode node = null;
            string filename = KSPUtil.ApplicationRootPath + "GameData/AtmosphereAutopilot/" + currentVessel.vesselName + ".cfg";
            ConfigNode fileNode = ConfigNode.Load(filename);
            if (fileNode != null)
            {
                var nodes = fileNode.GetNodes(NodeName);
                try
                {
                    node = nodes != null ? nodes.First() : null;
                }
                catch { node = null; }
                if (node != null)
                {
                    loadPIDfromNode(node);
                    Debug.Log("[Autopilot]: Dampener preset was loaded from file");
                    return true;
                }
            }
            Debug.Log("[Autopilot]: node wasn't found in loadFromPreset call");
            return false;
        }

        protected void loadGlobalData(string NodeName)
        {
            ConfigNode node = null;
            string filename = KSPUtil.ApplicationRootPath + "GameData/AtmosphereAutopilot/UI.cfg";
            ConfigNode fileNode = ConfigNode.Load(filename);
            if (fileNode != null)
            {
                var nodes = fileNode.GetNodes(NodeName);
                try
                {
                    node = nodes != null ? nodes.First() : null;
                }
                catch { node = null; }
                if (node != null)
                {
                    string str;
                    if ((str = node.GetValue("window_xMin")) != null)
                        window.xMin = float.Parse(str);
                    if ((str = node.GetValue("window_yMin")) != null)
                        window.yMin = float.Parse(str);
                    if ((str = node.GetValue("window_width")) != null)
                        window.width = float.Parse(str);
                    if ((str = node.GetValue("window_height")) != null)
                        window.height = float.Parse(str);
                }
            }
        }

        public abstract void serialize();

        protected virtual void saveToFile(string NodeName)
        {
            Debug.Log("[Autopilot]: saving " + NodeName + " for " + currentVessel.vesselName + " to file");

            string filename = KSPUtil.ApplicationRootPath + "GameData/AtmosphereAutopilot/" + currentVessel.vesselName + ".cfg";
            ConfigNode fileNode = ConfigNode.Load(filename);
            if (fileNode == null)
                fileNode = new ConfigNode();
            fileNode.RemoveNode(NodeName);

            ConfigNode node = new ConfigNode(NodeName);
            node.AddValue("vessel_name", currentVessel.vesselName);
            node.AddValue("KP", pid.KP);
            node.AddValue("KD", pid.KD);
            node.AddValue("KI", pid.KI);
            node.AddValue("IntegralClamp", pid.IntegralClamp);
            node.AddValue("AccumulDerivClamp", pid.AccumulDerivClamp);
            node.AddValue("AccumulatorClamp", pid.AccumulatorClamp);

            fileNode.AddNode(node);
            fileNode.Save(filename);

            string filename_global = KSPUtil.ApplicationRootPath + "GameData/AtmosphereAutopilot/UI.cfg";
            ConfigNode fileGNode = ConfigNode.Load(filename_global);
            if (fileGNode == null)
                fileGNode = new ConfigNode();
            fileGNode.RemoveNode(NodeName);

            ConfigNode gnode = new ConfigNode(NodeName);
            gnode.AddValue("window_xMin", window.xMin);
            gnode.AddValue("window_yMin", window.yMin);
            gnode.AddValue("window_width", window.width);
            gnode.AddValue("window_height", window.height);            

            fileGNode.AddNode(gnode);
            fileGNode.Save(filename_global);
        }

        protected void loadPIDfromNode(ConfigNode node)
        {
            string str;
            if ((str = node.GetValue("KP")) != null)
                pid.KP = double.Parse(str);
            if ((str = node.GetValue("KD")) != null)
                pid.KD = double.Parse(str);
            if ((str = node.GetValue("KI")) != null)
                pid.KI = double.Parse(str);
            if ((str = node.GetValue("IntegralClamp")) != null)
                pid.IntegralClamp = double.Parse(str);
            if ((str = node.GetValue("AccumulDerivClamp")) != null)
                pid.AccumulDerivClamp = double.Parse(str);
            if ((str = node.GetValue("AccumulatorClamp")) != null)
                pid.AccumulatorClamp = double.Parse(str);
        }

        protected PIDController pid;
        protected double angular_velocity;
        protected double output;

        public virtual void Activate()
        {
            currentVessel.OnAutopilotUpdate += new FlightInputCallback(OnFixedUpdate);
            enabled = true;
        }

        public virtual void Deactivate()
        {
            currentVessel.OnAutopilotUpdate -= new FlightInputCallback(OnFixedUpdate);
            enabled = false;
        }

        public bool gui_shown = false;
        public void toggleGUI()
        {
            gui_shown = !gui_shown;
            if (gui_shown)
            {
                kp_str = pid.KP.ToString("G8");
                ki_str = pid.KI.ToString("G8");
                kd_str = pid.KD.ToString("G8");
                ic_str = pid.IntegralClamp.ToString("G8");
                ac_str = pid.AccumulatorClamp.ToString("G8");
                adc_str = pid.AccumulDerivClamp.ToString("G8");
            }
        }

        protected string damper_name;
        protected int wnd_id;
        protected Rect window = new Rect(50.0f, 100.0f, 230.0f, 250.0f);

        public void drawGUI()
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
