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
using UnityEngine;

namespace AtmosphereAutopilot
{

    public struct Waypoint
    {
        public Waypoint(double longt, double lat)
        {
            longtitude = longt;
            latitude = lat;
        }
        public double longtitude;
        public double latitude;
    }

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

        Vector3d desired_velocity = Vector3d.zero;
        Vector3d planet2ves = Vector3d.zero;
        Vector3d planet2vesNorm = Vector3d.zero;
        Vector3d desired_vert_acc = Vector3d.zero;

        // centrifugal acceleration to stay on desired altitude
        Vector3d level_acc = Vector3d.zero;

        public override void ApplyControl(FlightCtrlState cntrl)
        {
            if (vessel.LandedOrSplashed)
                return;

            if (speed_control)
                thrust_c.ApplyControl(cntrl, desired_spd);

            desired_velocity = Vector3d.zero;
            planet2ves = vessel.ReferenceTransform.position - vessel.mainBody.position;
            planet2vesNorm = planet2ves.normalized;
            desired_vert_acc = Vector3d.zero;

            // centrifugal acceleration to stay on desired altitude
            level_acc = -planet2vesNorm * (imodel.surface_v - Vector3d.Project(imodel.surface_v, planet2vesNorm)).sqrMagnitude / planet2ves.magnitude;

            switch (current_mode)
            {
                default:
                case CruiseMode.LevelFlight:
                    // simply select velocity from axis
                    desired_velocity = Vector3d.Cross(planet2vesNorm, circle_axis);
                    handle_wide_turn();
                    if (specific_altitude)
                        desired_velocity = account_for_height(desired_velocity);
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
                    handle_wide_turn();
                    if (specific_altitude)
                        desired_velocity = account_for_height(desired_velocity);
                    break;

                case CruiseMode.Waypoint:
                    if (!waypoint_entered)
                    {
                        // goto simple level flight
                        goto case CruiseMode.LevelFlight;
                    }
                    else
                    {
                        // set new axis
                        Vector3d world_target_pos = vessel.mainBody.GetWorldSurfacePosition(current_waypt.latitude,
                            current_waypt.longtitude, vessel.altitude);
                        double dist_to_dest = Vector3d.Distance(world_target_pos, vessel.ReferenceTransform.position);
                        if (dist_to_dest < 200.0)
                        {
                            // we're too close to target, let's switch to level flight
                            LevelFlightMode = true;
                            picking_waypoint = false;
                            MessageManager.post_quick_message("Waypoint reached");
                            goto case CruiseMode.LevelFlight;
                        }
                        // set new axis according to waypoint
                        circle_axis = Vector3d.Cross(world_target_pos - vessel.mainBody.position, vessel.GetWorldPos3D() - vessel.mainBody.position).normalized;
                        goto case CruiseMode.LevelFlight;
                    }
            }

            double old_str = dir_c.strength;
            dir_c.strength *= strength_mult;
            dir_c.ApplyControl(cntrl, desired_velocity, level_acc + desired_vert_acc);
            dir_c.strength = old_str;
        }

        void handle_wide_turn()
        {
            if (Vector3d.Dot(imodel.surface_v, desired_velocity) < 0.0)
            {
                // we're turning for more than 90 degrees, let's force the turn to be horizontal
                Vector3d right_turn = Vector3d.Cross(planet2vesNorm, imodel.surface_v);
                double sign = Math.Sign(Vector3d.Dot(right_turn, desired_velocity));
                if (sign == 0.0)
                    sign = 1.0;
                desired_velocity = right_turn * sign;
            }
        }

        public enum CruiseMode
        {
            LevelFlight,
            CourseHold,
            Waypoint
        }

        public CruiseMode current_mode = CruiseMode.LevelFlight;

        public Waypoint current_waypt = new Waypoint();
        bool waypoint_entered = false;

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
        [AutoGuiAttr("Hold specific altitude", true)]
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
        public double strength_mult = 0.75;

        [VesselSerializable("height_relax_time")]
        [AutoGuiAttr("height_relax_time", true, "G5")]
        public double height_relax_time = 6.0;

        [VesselSerializable("height_relax_Kp")]
        [AutoGuiAttr("height_relax_Kp", true, "G5")]
        public double height_relax_Kp = 0.3;

        [VesselSerializable("max_climb_angle")]
        [AutoGuiAttr("max_climb_angle", true, "G5")]
        public double max_climb_angle = 20.0;

        Vector3d account_for_height(Vector3d desired_direction)
        {
            double cur_alt = vessel.altitude;
            double height_error = desired_altitude - cur_alt;
            double acc = Vector3.Dot(imodel.gravity_acc + imodel.noninert_acc, -planet2vesNorm);    // free-fall vertical acceleration
            double height_relax_frame = 0.5 * acc * height_relax_time * height_relax_time;

            double relax_transition_k = 0.0;
            double des_vert_speed = 0.0;
            double relax_vert_speed = 0.0;
            Vector3d res = Vector3d.zero;

            Vector3d proportional_acc = Vector3d.zero;
            double cur_vert_speed = Vector3d.Dot(imodel.surface_v, planet2vesNorm);
            if (Math.Abs(height_error) < height_relax_frame)
            {
                relax_transition_k = Common.Clamp(2.0 * (height_relax_frame - Math.Abs(height_error)), 0.0, 1.0);
                // we're in relaxation frame
                relax_vert_speed = height_relax_Kp * height_error;
                // exponential descent                
                if (cur_vert_speed * height_error > 0.0)
                    proportional_acc = -planet2vesNorm * height_relax_Kp * cur_vert_speed;
            }
            
            // let's assume parabolic ascent\descend
            Vector3d parabolic_acc = Vector3d.zero;
            if (height_error >= 0.0)
            {
                des_vert_speed = Math.Sqrt(acc * height_error);
                if (cur_vert_speed > 0.0)
                    parabolic_acc = -planet2vesNorm * 0.5 * cur_vert_speed * cur_vert_speed / height_error;
            }
            else
            {
                double vert_acc_descent = 2.0 * Math.Min(-5.0, acc - dir_c.strength * strength_mult * dir_c.max_lift_acc * 0.5);
                des_vert_speed = -Math.Sqrt(vert_acc_descent * height_error);
                if (cur_vert_speed < 0.0)
                    parabolic_acc = -planet2vesNorm * 0.5 * cur_vert_speed * cur_vert_speed / height_error;
            }
            double max_vert_speed = vessel.horizontalSrfSpeed * Math.Tan(max_climb_angle * dgr2rad);
            bool apply_acc = Math.Abs(des_vert_speed) < max_vert_speed;
            des_vert_speed = Common.Clamp(des_vert_speed, max_vert_speed);
            res = desired_direction.normalized * vessel.horizontalSrfSpeed + planet2vesNorm * Common.lerp(des_vert_speed, relax_vert_speed, relax_transition_k);
            if (apply_acc)
                desired_vert_acc = parabolic_acc * (1.0 - relax_transition_k) + proportional_acc * relax_transition_k;
            return res.normalized;
        }



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
                        waypoint_entered = false;
                        circle_axis = Vector3d.Cross(vessel.srf_velocity, vessel.GetWorldPos3D() - vessel.mainBody.position).normalized;
                        start_picking_waypoint();
                    }
                    current_mode = CruiseMode.Waypoint;
                }
            }
        }

        void start_picking_waypoint()
        {
            MapView.EnterMapView();
            MessageManager.post_quick_message("Pick destination");
            picking_waypoint = true;            
        }

        bool picking_waypoint = false;

        protected override void _drawGUI(int id)
        {
            GUILayout.BeginVertical();
            GUILayout.BeginHorizontal();
            // three buttons to switch mode
            LevelFlightMode = GUILayout.Toggle(LevelFlightMode, "Level", GUIStyles.toggleButtonStyle);
            CourseHoldMode = GUILayout.Toggle(CourseHoldMode, "Course", GUIStyles.toggleButtonStyle);
            WaypointMode = GUILayout.Toggle(WaypointMode, "Waypoint", GUIStyles.toggleButtonStyle);
            GUILayout.EndHorizontal();
            // waypoint picking button
            if (WaypointMode)
            {
                if (GUILayout.Button("pick waypoint", GUIStyles.toggleButtonStyle) && !picking_waypoint)
                    start_picking_waypoint();
                GUILayout.BeginHorizontal();
                GUILayout.Label(current_waypt.latitude.ToString("G5"), GUIStyles.labelStyleCenter);
                GUILayout.Label(current_waypt.longtitude.ToString("G5"), GUIStyles.labelStyleCenter);
                GUILayout.EndHorizontal();
            }
            AutoGUI.AutoDrawObject(this);
            GUILayout.EndVertical();
            GUI.DragWindow();
        }

        public override void OnUpdate()
        {
            if (picking_waypoint)
            {
                if (!HighLogic.LoadedSceneIsFlight || !MapView.MapIsEnabled)
                {
                    // we leaved map without picking
                    MessageManager.post_quick_message("Cancelled");
                    picking_waypoint = false;
                    return;
                }
                // Thanks MechJeb!
                if (Input.GetMouseButtonDown(0) && !window.Contains(Input.mousePosition))
                {
                    Ray mouseRay = PlanetariumCamera.Camera.ScreenPointToRay(Input.mousePosition);
                    mouseRay.origin = ScaledSpace.ScaledToLocalSpace(mouseRay.origin);
                    Vector3d relOrigin = mouseRay.origin - vessel.mainBody.position;
                    Vector3d relSurfacePosition;
                    double curRadius = vessel.mainBody.pqsController.radiusMax;
                    if (PQS.LineSphereIntersection(relOrigin, mouseRay.direction, curRadius, out relSurfacePosition))
                    {
                        Vector3d surfacePoint = vessel.mainBody.position + relSurfacePosition;
                        current_waypt.longtitude = vessel.mainBody.GetLongitude(surfacePoint);
                        current_waypt.latitude = vessel.mainBody.GetLatitude(surfacePoint);
                        picking_waypoint = false;
                        waypoint_entered = true;
                        MessageManager.post_quick_message("Picked");
                    }
                    else
                    {
                        MessageManager.post_quick_message("Missed");
                    }
                }
            }
        }
    }
}
