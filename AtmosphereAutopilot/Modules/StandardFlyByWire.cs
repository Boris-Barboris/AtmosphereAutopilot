using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;

namespace AtmosphereAutopilot
{
    public sealed class StandardFlyByWire : StateController
    {
        PitchAngularVelocityController pc;
        RollAngularVelocityController rc;
        SideslipController yc;
        AutopilotModule[] gui_list = new AutopilotModule[3];

        internal StandardFlyByWire(Vessel v) :
            base(v, "Standard Fly-By-Wire", 44421322) { }

        public override void InitializeDependencies(Dictionary<Type, AutopilotModule> modules)
        {
            gui_list[0] = pc = modules[typeof(PitchAngularVelocityController)] as PitchAngularVelocityController;
            gui_list[1] = rc = modules[typeof(RollAngularVelocityController)] as RollAngularVelocityController;
            gui_list[2] = yc = modules[typeof(SideslipController)] as SideslipController;
        }

        protected override void OnActivate() 
        {
            pc.Activate();
            rc.Activate();
            yc.Activate();
        }

        protected override void OnDeactivate()
        {
            pc.Deactivate();
            rc.Deactivate();
            yc.Deactivate();
        }

        /// <summary>
        /// Main control function
        /// </summary>
        /// <param name="cntrl">Control state to change</param>
        public override void ApplyControl(FlightCtrlState cntrl)
        {
            if (vessel.LandedOrSplashed)
                return;

            pc.ApplyControl(cntrl, 0.0f);
            yc.ApplyControl(cntrl, 0.0f);
            rc.ApplyControl(cntrl, 0.0f);            
        }

        protected override void _drawGUI(int id)
        {
            GUILayout.BeginVertical();
            foreach (var module in gui_list)
            {
                bool is_shown = GUILayout.Toggle(module.IsShown(), module.ModuleName + " GUI", GUIStyles.toggleButtonStyle);
                if (is_shown)
                    module.ShowGUI();
                else
                    module.UnShowGUI();
            }
            GUILayout.EndVertical();
            GUI.DragWindow();
        }
    }
}
