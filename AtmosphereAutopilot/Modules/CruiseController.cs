using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;

namespace AtmosphereAutopilot
{
    public sealed class CruiseController: StateController
    {
        internal CruiseController(Vessel v)
            : base(v, "Cruise Flight", 88437226)
        { }

        FlightModel imodel;
        AccelerationController acc_c;

        public override void InitializeDependencies(Dictionary<Type, AutopilotModule> modules)
        {
            imodel = modules[typeof(FlightModel)] as FlightModel;
            acc_c = modules[typeof(AccelerationController)] as AccelerationController;
        }

        protected override void OnActivate()
        {
            acc_c.Activate();   
        }

        protected override void OnDeactivate()
        {
            acc_c.Deactivate();
        }

        public override void ApplyControl(FlightCtrlState cntrl)
        {
            Vector3d desired_acc = Vector3d.zero;
            Vector3d planet2ves = vessel.ReferenceTransform.position - vessel.mainBody.position;
            Vector3d planet2vesNorm = planet2ves.normalized;

            // acceleration to stay on desired altitude
            desired_acc -= planet2vesNorm * (imodel.surface_v - Vector3d.Project(imodel.surface_v, planet2vesNorm)).sqrMagnitude / planet2ves.magnitude;

            // right acceleration
            Vector3d right_v = Vector3d.Cross(planet2vesNorm, imodel.surface_v.normalized).normalized;
            desired_acc += right_v * right_acceleration;

            // up acceleration
            desired_acc += planet2vesNorm * up_acceleration;

            acc_c.ApplyControl(cntrl, desired_acc);
        }

        [AutoGuiAttr("Acceleration controller GUI", true)]
        protected bool AccCGUI { get { return acc_c.IsShown(); } set { if (value) acc_c.ShowGUI(); else acc_c.UnShowGUI(); } }

        [AutoGuiAttr("right_acceleration", true, "G5")]
        protected double right_acceleration = 0.0;

        [AutoGuiAttr("up_acceleration", true, "G5")]
        protected double up_acceleration = 0.0;
    }
}
