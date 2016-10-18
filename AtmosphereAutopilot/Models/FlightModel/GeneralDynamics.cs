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
        public double prev_pitch_gravity_acc;

        [AutoGuiAttr("pitch_engine_acc", false, "G6")]
        public double pitch_engine_acc;

        [AutoGuiAttr("pitch_noninert_acc", false, "G6")]
        public double pitch_noninert_acc;
        public double prev_pitch_noninert_acc;

        [AutoGuiAttr("yaw_gravity_acc", false, "G6")]
        public double yaw_gravity_acc;
        public double prev_yaw_gravity_acc;

        [AutoGuiAttr("yaw_engine_acc", false, "G6")]
        public double yaw_engine_acc;

        [AutoGuiAttr("yaw_noninert_acc", false, "G6")]
        public double yaw_noninert_acc;
        public double prev_yaw_noninert_acc;

        //[AutoGuiAttr("pitch_tangent", false, "G6")]
        public Vector3d pitch_tangent;
        public Vector3d prev_pitch_tangent;

        //[AutoGuiAttr("yaw_tangent", false, "G6")]
        public Vector3d yaw_tangent;
        public Vector3d prev_yaw_tangent;

        public Vector3d prev_surface_v;
        //Vector3d prev_gravity_acc;
        //Vector3d prev_noninert_acc;
        float prev_mass = 1.0f;
        public Quaternion prev_cntrl2world = Quaternion.identity;
        //Vector3 prev_right;
        //Vector3 prev_forward;

        void update_dynamics()
        {
            dyn_pressure = vessel.atmDensity * vessel.srfSpeed * vessel.srfSpeed;

            gravity_acc = FlightGlobals.getGeeForceAtPosition(CoM);
            noninert_acc = FlightGlobals.getCoriolisAcc(surface_v, vessel.mainBody) +
                FlightGlobals.getCentrifugalAcc(CoM, vessel.mainBody);

            sum_acc = (surface_v - prev_surface_v) / TimeWarp.fixedDeltaTime;

            Vector3 prev_thrust_world = prev_cntrl2world * engines_thrust_principal;

            // pitch
            pitch_tangent = Vector3d.Cross(surface_v.normalized, vessel.ReferenceTransform.right).normalized;
            double pitch_total_acc = Vector3d.Dot(sum_acc, prev_pitch_tangent);
            pitch_gravity_acc = Vector3d.Dot(gravity_acc, prev_pitch_tangent);
            pitch_noninert_acc = Vector3d.Dot(noninert_acc, prev_pitch_tangent);
            pitch_engine_acc = Vector3d.Dot(prev_thrust_world / prev_mass, prev_pitch_tangent);
            lift_acc = pitch_total_acc - prev_pitch_noninert_acc - prev_pitch_gravity_acc - pitch_engine_acc;

            // yaw
            yaw_tangent = Vector3d.Cross(surface_v.normalized, vessel.ReferenceTransform.forward).normalized;
            double yaw_total_acc = Vector3d.Dot(sum_acc, prev_yaw_tangent);
            yaw_gravity_acc = Vector3d.Dot(gravity_acc, prev_yaw_tangent);
            yaw_noninert_acc = Vector3d.Dot(noninert_acc, prev_yaw_tangent);
            yaw_engine_acc = Vector3d.Dot(prev_thrust_world / prev_mass, prev_yaw_tangent);
            slide_acc = yaw_total_acc - prev_yaw_noninert_acc - prev_yaw_gravity_acc - yaw_engine_acc;
        }

        void postupdate_dynamics()
        {
            // update previous states with current values
            prev_surface_v = surface_v;
            //prev_noninert_acc = noninert_acc;
            //prev_gravity_acc = gravity_acc;
            prev_mass = sum_mass;
            prev_cntrl2world = cntrl_part_to_world;
            //prev_right = vessel.ReferenceTransform.right;
            //prev_forward = vessel.ReferenceTransform.forward;
            prev_pitch_tangent = pitch_tangent;
            prev_yaw_tangent = yaw_tangent;
            prev_pitch_gravity_acc = pitch_gravity_acc;
            prev_pitch_noninert_acc = pitch_noninert_acc;
            prev_yaw_gravity_acc = yaw_gravity_acc;
            prev_yaw_noninert_acc = yaw_noninert_acc;
        }
    }
}
