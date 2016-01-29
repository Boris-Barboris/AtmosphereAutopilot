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
using UnityEngine;

namespace AtmosphereAutopilot
{

    /// <summary>
    /// Manages cruise flight modes, like heading and altitude holds
    /// </summary>
    public sealed class CruiseController: StateController
    {
        internal CruiseController(Vessel v)
            : base(v, "Cruise Flight controller", 88437226)
        { }

        FlightModel imodel;
        DirectorController dir_c;
        ProgradeThrustController thrust_c;

        public override void InitializeDependencies(Dictionary<Type, AutopilotModule> modules)
        {
            imodel = modules[typeof(FlightModel)] as FlightModel;
            dir_c = modules[typeof(DirectorController)] as DirectorController;
            thrust_c = modules[typeof(ProgradeThrustController)] as ProgradeThrustController;
        }

        protected override void OnActivate()
        {
            dir_c.Activate();
            thrust_c.Activate();
            imodel.Activate();
            MessageManager.post_status_message("Cruise Flight enabled");
            
            // let's set new circle axis
            circle_axis = Vector3d.Cross(vessel.srf_velocity, vessel.GetWorldPos3D() - vessel.mainBody.position).normalized;
        }

        protected override void OnDeactivate()
        {
            dir_c.Deactivate();
            thrust_c.Deactivate();
            imodel.Deactivate();
            MessageManager.post_status_message("Cruise Flight disabled");
        }

        public override void ApplyControl(FlightCtrlState cntrl)
        {
            if (vessel.LandedOrSplashed)
                return;

            if (speed_control)
                thrust_c.ApplyControl(cntrl, desired_spd);

            Vector3d desired_velocity = Vector3d.zero;
            Vector3d planet2ves = vessel.ReferenceTransform.position - vessel.mainBody.position;
            Vector3d planet2vesNorm = planet2ves.normalized;

            // centrifugal acceleration to stay on desired altitude
            Vector3d level_acc = -planet2vesNorm * (imodel.surface_v - Vector3d.Project(imodel.surface_v, planet2vesNorm)).sqrMagnitude / planet2ves.magnitude;

            switch (current_mode)
            {
                case CruiseMode.LevelFlight:
                    // simply select velocity from axis
                    desired_velocity = Vector3d.Cross(planet2vesNorm, circle_axis);
                    break;
                case CruiseMode.CourseHold:
                    if (Math.Abs(vessel.latitude) > 80.0)
                    {
                        // we're too close to poles, let's switch to level flight
                        LevelFlightMode = true;
                        goto case CruiseMode.LevelFlight;
                    }
                    // get direction vector form course
                    Vector3d north = vessel.mainBody.RotationAxis;
                    Vector3d north_projected = Vector3.ProjectOnPlane(north, planet2vesNorm);
                    QuaternionD rotation = QuaternionD.AngleAxis(desired_course, planet2vesNorm);
                    desired_velocity = rotation * north_projected;
                    if (Vector3d.Dot(imodel.surface_v, desired_velocity) < 0.0)
                    {
                        // we're turning for more than 90 degrees, let's force the turn to be horizontal
                        Vector3d right_turn = Vector3d.Cross(planet2vesNorm, imodel.surface_v);
                        double sign = Math.Sign(Vector3d.Dot(right_turn, desired_velocity));
                        if (sign == 0.0)
                            sign = 1.0;
                        desired_velocity = right_turn * sign;
                    }
                    break;
                default:
                    break;
            }

            double old_str = dir_c.strength;
            dir_c.strength *= strength_mult;
            dir_c.ApplyControl(cntrl, desired_velocity, level_acc);
            dir_c.strength = old_str;
        }

        public enum CruiseMode
        {
            LevelFlight,
            CourseHold,
            Waypoint
        }

        public CruiseMode current_mode = CruiseMode.LevelFlight;

        // axis to rotate around in level flight mode
        protected Vector3d circle_axis = Vector3d.zero;

        [AutoGuiAttr("Director controller GUI", true)]
        protected bool DircGUI { get { return dir_c.IsShown(); } set { if (value) dir_c.ShowGUI(); else dir_c.UnShowGUI(); } }

        [AutoGuiAttr("Thrust controller GUI", true)]
        protected bool PTCGUI { get { return thrust_c.IsShown(); } set { if (value) thrust_c.ShowGUI(); else thrust_c.UnShowGUI(); } }

        [VesselSerializable("desired_course")]
        [AutoGuiAttr("desired_course", true)]
        public float desired_course = 90.0f;

        [VesselSerializable("specific_altitude")]
        [AutoGuiAttr("Specific altitude", true)]
        public bool specific_altitude = false;

        [VesselSerializable("desired_altitude")]
        [AutoGuiAttr("desired_altitude", true)]
        public float desired_altitude = 1000.0f;

        [VesselSerializable("speed_control")]
        [AutoGuiAttr("Speed control", true)]
        public bool speed_control = false;

        [VesselSerializable("cruise_speed")]
        [GlobalSerializable("cruise_speed")]
        [AutoGuiAttr("Cruise speed", true, "G5")]
        public float desired_spd = 200.0f;

        [VesselSerializable("strength_mult")]
        [AutoGuiAttr("strength_mult", true, "G5")]
        public double strength_mult = 0.5;

        [AutoGuiAttr("latitude", false, "G5")]
        public double latitude { get { return vessel.latitude; } }

        bool LevelFlightMode
        {
            get { return current_mode == CruiseMode.LevelFlight; }
            set
            {
                if (value)
                {
                    if (current_mode != CruiseMode.LevelFlight)
                    {
                        // let's set new circle axis
                        circle_axis = Vector3d.Cross(vessel.srf_velocity, vessel.GetWorldPos3D() - vessel.mainBody.position).normalized;
                    }
                    current_mode = CruiseMode.LevelFlight;
                }
            }
        }

        bool CourseHoldMode
        {
            get { return current_mode == CruiseMode.CourseHold; }
            set
            {
                if (value)
                {
                    if (Math.Abs(vessel.latitude) > 80.0)
                        return;
                    if (current_mode != CruiseMode.CourseHold)
                    {
                        // TODO
                    }
                    current_mode = CruiseMode.CourseHold;
                }
            }
        }

        bool WaypointMode
        {
            get { return current_mode == CruiseMode.Waypoint; }
            set
            {
                if (value)
                {
                    if (current_mode != CruiseMode.Waypoint)
                    {
                        // TODO
                    }
                    current_mode = CruiseMode.Waypoint;
                }
            }
        }

        protected override void _drawGUI(int id)
        {
            GUILayout.BeginVertical();
            GUILayout.BeginHorizontal();
            // three buttons to switch mode
            LevelFlightMode = GUILayout.Toggle(LevelFlightMode, "Level", GUIStyles.toggleButtonStyle);
            CourseHoldMode = GUILayout.Toggle(CourseHoldMode, "Course", GUIStyles.toggleButtonStyle);
            WaypointMode = GUILayout.Toggle(WaypointMode, "Waypoint", GUIStyles.toggleButtonStyle);
            GUILayout.EndHorizontal();
            GUILayout.Space(5.0f);
            AutoGUI.AutoDrawObject(this);
            GUILayout.EndVertical();
            GUI.DragWindow();
        }
    }
}
