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
    abstract class Damper
    {
        protected Vessel currentVessel = null;
        protected bool enabled = false;
        public bool Enabled { get { return enabled; } }

        public Damper(Vessel cur_vessel)
        {
            currentVessel = cur_vessel;
        }

        public virtual void Activate()
        {
            currentVessel.OnAutopilotUpdate += new FlightInputCallback(apply_module);
            enabled = true;
        }

        public virtual void Deactivate()
        {
            currentVessel.OnAutopilotUpdate -= new FlightInputCallback(apply_module);
            enabled = false;
        }

        public abstract void toggleGUI();

        protected abstract void drawGUI();

        protected abstract void apply_module(FlightCtrlState cntrl);
    }
}
