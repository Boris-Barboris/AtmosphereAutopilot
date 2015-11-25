/*
Atmosphere Autopilot, plugin for Kerbal Space Program.
Copyright (C) 2015, Baranin Alexander aka Boris-Barboris.
 
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
using System.Threading;
using UnityEngine;
using System.IO;

namespace AtmosphereAutopilot
{

    using Vector = VectorArray.Vector;

    public sealed partial class FlightModel : AutopilotModule
	{
        [AutoGuiAttr("Lift acc", false, "G6")]
        public double lift_acc = 0.0;

        [AutoGuiAttr("Slide acc", false, "G6")]
        public double slide_acc = 0.0;

        //Vector3d vess2planet;
        //[AutoGuiAttr("gravity_acc acc", false, "G6")]
        public Vector3d gravity_acc;

        //[AutoGuiAttr("noninert_acc", false, "G6")]
        public Vector3d noninert_acc;

        [AutoGuiAttr("sum_acc", false, "G3")]
        public Vector3d sum_acc;

        [AutoGuiAttr("pitch_gravity_acc", false, "G6")]
        public double pitch_gravity_acc;

        [AutoGuiAttr("pitch_engine_acc", false, "G6")]
        public double pitch_engine_acc;

        [AutoGuiAttr("pitch_noninert_acc", false, "G6")]
        public double pitch_noninert_acc;

        [AutoGuiAttr("yaw_gravity_acc", false, "G6")]
        public double yaw_gravity_acc;

        [AutoGuiAttr("yaw_engine_acc", false, "G6")]
        public double yaw_engine_acc;

        [AutoGuiAttr("yaw_noninert_acc", false, "G6")]
        public double yaw_noninert_acc;

        //[AutoGuiAttr("pitch_tangent", false, "G6")]
        public Vector3d pitch_tangent;

        //[AutoGuiAttr("yaw_tangent", false, "G6")]
        public Vector3d yaw_tangent;

        Vector3d prev_orb_vel;

        void update_dynamics()
        {
            dyn_pressure = vessel.atmDensity * vessel.srfSpeed * vessel.srfSpeed;

            //vess2planet = vessel.mainBody.position - vessel.ReferenceTransform.position;
            //gravity_acc = vess2planet.normalized * (vessel.mainBody.gMagnitudeAtCenter / vess2planet.sqrMagnitude);
            gravity_acc = integrator.gForce;
            noninert_acc = integrator.CoriolisAcc + integrator.CentrifugalAcc;

            Vector3d orb_vel = surface_v;

            sum_acc = (orb_vel - prev_orb_vel) / TimeWarp.fixedDeltaTime;
            prev_orb_vel = orb_vel;

            // pitch
            pitch_tangent = Vector3d.Cross(surface_v.normalized, vessel.ReferenceTransform.right).normalized;
            double pitch_total_acc = Vector3d.Dot(sum_acc, pitch_tangent);
            pitch_gravity_acc = Vector3d.Dot(gravity_acc, pitch_tangent);
            pitch_noninert_acc = Vector3d.Dot(noninert_acc, pitch_tangent);
            pitch_engine_acc = Vector3d.Dot(cntrl_part_to_world * engines_thrust_principal / sum_mass, pitch_tangent);
            lift_acc = pitch_total_acc - pitch_noninert_acc - pitch_gravity_acc - pitch_engine_acc;

            // yaw
            yaw_tangent = Vector3d.Cross(surface_v.normalized, vessel.ReferenceTransform.forward).normalized;
            double yaw_total_acc = Vector3d.Dot(sum_acc, yaw_tangent);
            yaw_gravity_acc = Vector3d.Dot(gravity_acc, yaw_tangent);
            yaw_noninert_acc = Vector3d.Dot(noninert_acc, yaw_tangent);
            yaw_engine_acc = Vector3d.Dot(cntrl_part_to_world * engines_thrust_principal / sum_mass, yaw_tangent);
            slide_acc = yaw_total_acc - yaw_noninert_acc - yaw_gravity_acc - yaw_engine_acc;
        }
    }
}
