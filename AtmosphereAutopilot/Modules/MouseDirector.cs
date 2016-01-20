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
    public sealed class MouseDirector: StateController
    {
        internal MouseDirector(Vessel v)
            : base(v, "Mouse Director", 88437227)
        {}

        FlightModel imodel;
        AccelerationController acc_c;
        ProgradeThrustController thrust_c;

        public override void InitializeDependencies(Dictionary<Type, AutopilotModule> modules)
        {
            imodel = modules[typeof(FlightModel)] as FlightModel;
            acc_c = modules[typeof(AccelerationController)] as AccelerationController;
            thrust_c = modules[typeof(ProgradeThrustController)] as ProgradeThrustController;
        }

        protected override void OnActivate()
        {
            acc_c.Activate();
            thrust_c.Activate();
            MessageManager.post_status_message("Mouse Director enabled");
        }

        protected override void OnDeactivate()
        {
            acc_c.Deactivate();
            thrust_c.Deactivate();
            if (indicator != null)
                indicator.enabled = false;
            MessageManager.post_status_message("Mouse Director disabled");
        }

        public override void ApplyControl(FlightCtrlState cntrl)
        {
            if (vessel.LandedOrSplashed)
                return;

            Vector3d desired_acc = Vector3d.zero;
            Vector3d planet2ves = vessel.ReferenceTransform.position - vessel.mainBody.position;
            Vector3d planet2vesNorm = planet2ves.normalized;

            // acceleration to stay on desired altitude
            desired_acc -= planet2vesNorm * (imodel.surface_v - Vector3d.Project(imodel.surface_v, planet2vesNorm)).sqrMagnitude / planet2ves.magnitude;

            //
            // follow camera direction
            //
            Vector3d desired_turn_acc_dir = Vector3d.Cross(
                Vector3d.Cross(imodel.surface_v.normalized, camera_direction).normalized,
                imodel.surface_v.normalized);
            angular_error = Vector3d.Angle(imodel.surface_v.normalized, camera_direction) * dgr2rad;
            double max_lift = acc_c.max_lift_acceleration(PITCH);
            
            // let's find this craft's maximum acceleration toward desired_turn_acc_dir without stalling
            // it can be solved from simple quadratic equation
            Vector3d max_turn_acc = Vector3d.zero;
            Vector3d rest_lift = imodel.pitch_tangent * (-imodel.pitch_gravity_acc - imodel.pitch_noninert_acc);
            double b = 2.0 * Vector3d.Dot(rest_lift, desired_turn_acc_dir);
            double a = desired_turn_acc_dir.sqrMagnitude;
            double c = rest_lift.sqrMagnitude - max_lift * max_lift;
            if (Math.Abs(a) > 1e-10)
            {
                double discriminant = b * b - 4.0 * a * c;
                if (discriminant < 0.0)
                {
                    max_turn_acc = max_lift * desired_turn_acc_dir;
                }
                else
                {
                    double disc_root = Math.Sqrt(discriminant);
                    double k = (-b + disc_root) / 2.0 / a;
                    max_turn_acc = desired_turn_acc_dir * k; 
                }
            }
            else
            {
                if (Math.Abs(b) > 1e-10)
                {
                    double k = -c / b;
                    max_turn_acc = desired_turn_acc_dir * k;
                }
                else
                    max_turn_acc = max_lift * desired_turn_acc_dir;
            }
            max_turn_acc = strength * max_turn_acc * (FlightInputHandler.fetch.precisionMode ? 0.4 : 1.0);

            // now let's take roll speed and relaxation into account
            max_angular_v = max_turn_acc.magnitude / imodel.surface_v_magnitude;
            double t1 = acc_c.max_roll_v / acc_c.roll_acc_factor;
            double t2 = (90.0 * dgr2rad - t1 * t1 * acc_c.roll_acc_factor) / t1 / acc_c.roll_acc_factor;
            stop_time_roll = roll_stop_k * (2.0 * t1 + t2);
            if (double.IsNaN(stop_time_roll) || double.IsInfinity(stop_time_roll))
                stop_time_roll = 2.0;

            // now let's generate desired acceleration
            if (angular_error / max_angular_v > stop_time_roll)
            {
                // we're far away from relaxation, let's simply produce maximum acceleration
                desired_acc += max_turn_acc;
            }
            else
            {
                // we're relaxing now, quadratic descend is good approximation
                {
                    double tk = (angular_error / max_angular_v) / stop_time_roll;
                    if (Math.Abs(angular_error) < relaxation_margin)
                        tk *= Mathf.Lerp(1.0f, angle_relaxation_k, (float)(1.0f - Math.Abs(angular_error) / relaxation_margin));
                    desired_acc += max_turn_acc * tk;
                }
            }

            acc_c.ApplyControl(cntrl, desired_acc, Vector3d.zero);

            if (cruise_control)
                thrust_c.ApplyControl(cntrl, desired_spd);
        }

        [AutoGuiAttr("strength", true, "G5")]
        public float strength = 0.95f;

        [AutoGuiAttr("roll_stop_k", true, "G5")]
        protected float roll_stop_k = 1.2f;

        [AutoGuiAttr("angular error", false, "G5")]
        protected double angular_error;

        [AutoGuiAttr("max_angular_v", false, "G5")]
        protected double max_angular_v;

        [AutoGuiAttr("stop_time_roll", false, "G5")]
        protected double stop_time_roll;

        [AutoGuiAttr("relaxation_margin", true, "G5")]
        protected double relaxation_margin = 0.01;

        [AutoGuiAttr("angle_relaxation_k", true, "G5")]
        protected float angle_relaxation_k = 0.1f;

        bool camera_correct = false;
        Vector3 camera_direction;

        static CenterIndicator indicator;
        static Camera camera_attached;

        public override void OnUpdate()
        {
            if (HighLogic.LoadedSceneIsFlight && CameraManager.Instance.currentCameraMode == CameraManager.CameraMode.Flight)
            {
                camera_correct = true;
                Camera maincamera = FlightCamera.fetch.mainCamera;
                camera_direction = maincamera.cameraToWorldMatrix.MultiplyPoint(new Vector3(0.0f, 0.0f, -1.0f)) -
                    FlightCamera.fetch.mainCamera.transform.position;
                // let's draw a couple of lines to show direction
                if (indicator == null || camera_attached != maincamera)
                {
                    indicator = maincamera.gameObject.GetComponent<CenterIndicator>();
                    if (indicator == null)
                        indicator = maincamera.gameObject.AddComponent<CenterIndicator>();
                    camera_attached = maincamera;
                }
                indicator.enabled = true;
            }
            else
            {
                camera_correct = false;
                indicator.enabled = false;
            }
        }

        [AutoGuiAttr("Acceleration controller GUI", true)]
        protected bool AccCGUI { get { return acc_c.IsShown(); } set { if (value) acc_c.ShowGUI(); else acc_c.UnShowGUI(); } }

        [AutoGuiAttr("Thrust controller GUI", true)]
        protected bool PTCGUI { get { return thrust_c.IsShown(); } set { if (value) thrust_c.ShowGUI(); else thrust_c.UnShowGUI(); } }

        [AutoGuiAttr("Cruise control", true)]
        public bool cruise_control = false;

        [VesselSerializable("cruise_speed")]
        [AutoGuiAttr("Cruise speed", true, "G5")]
        public float desired_spd = 100.0f;

        public class CenterIndicator: MonoBehaviour
        {
            Material mat = new Material(Shader.Find("KSP/Sprite"));

            Vector3 startVector = new Vector3(0.494f, 0.5f, -0.001f);
            Vector3 endVector = new Vector3(0.506f, 0.5f, -0.001f);

            public bool enabled = false;

            public void OnPostRender()
            {
                if (enabled)
                {
                    GL.PushMatrix();
                    mat.SetPass(0);
                    mat.color = Color.red;
                    GL.LoadOrtho();
                    GL.Begin(GL.LINES);
                    GL.Color(Color.red);
                    GL.Vertex(startVector);
                    GL.Vertex(endVector);
                    GL.End();
                    GL.PopMatrix();
                    enabled = false;
                }
            }
        }
    }
}
