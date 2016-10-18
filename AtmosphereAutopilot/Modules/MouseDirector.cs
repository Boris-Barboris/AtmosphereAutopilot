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
    public sealed class MouseDirector : StateController
    {
        internal MouseDirector(Vessel v)
            : base(v, "Mouse Director", 88437227)
        { }

        //FlightModel imodel;
        DirectorController dir_c;
        ProgradeThrustController thrust_c;

        public override void InitializeDependencies(Dictionary<Type, AutopilotModule> modules)
        {
            //imodel = modules[typeof(FlightModel)] as FlightModel;
            dir_c = modules[typeof(DirectorController)] as DirectorController;
            thrust_c = modules[typeof(ProgradeThrustController)] as ProgradeThrustController;
        }

        protected override void OnActivate()
        {
            dir_c.Activate();
            thrust_c.Activate();
            MessageManager.post_status_message("Mouse Director enabled");
        }

        protected override void OnDeactivate()
        {
            dir_c.Deactivate();
            thrust_c.Deactivate();
            if (indicator != null)
                indicator.enabled = false;
            MessageManager.post_status_message("Mouse Director disabled");
        }

        public override void ApplyControl(FlightCtrlState cntrl)
        {
            if (vessel.LandedOrSplashed)
                return;

            //
            // follow camera direction
            //
            dir_c.ApplyControl(cntrl, camera_direction, Vector3d.zero);

            if (thrust_c.spd_control_enabled)
                thrust_c.ApplyControl(cntrl, thrust_c.setpoint.mps());
        }

        //bool camera_correct = false;
        Vector3 camera_direction;

        static CenterIndicator indicator;
        static Camera camera_attached;

        public override void OnUpdate()
        {
            if (HighLogic.LoadedSceneIsFlight && CameraManager.Instance.currentCameraMode == CameraManager.CameraMode.Flight)
            {
                //camera_correct = true;
                Camera maincamera = FlightCamera.fetch.mainCamera;
                camera_direction = maincamera.cameraToWorldMatrix.MultiplyPoint(Vector3.back) -
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
                //camera_correct = false;
                indicator.enabled = false;
            }
        }

        [AutoGuiAttr("Director controller GUI", true)]
        public bool DirGUI { get { return dir_c.IsShown(); } set { if (value) dir_c.ShowGUI(); else dir_c.UnShowGUI(); } }

        [AutoGuiAttr("Thrust controller GUI", true)]
        public bool PTCGUI { get { return thrust_c.IsShown(); } set { if (value) thrust_c.ShowGUI(); else thrust_c.UnShowGUI(); } }

        protected override void _drawGUI(int id)
        {
            close_button();
            GUILayout.BeginVertical();
            AutoGUI.AutoDrawObject(this);
            thrust_c.SpeedCtrlGUIBlock();
            GUILayout.EndVertical();
            GUI.DragWindow();
        }

        public class CenterIndicator : MonoBehaviour
        {
            Material mat = new Material(Shader.Find("Sprites/Default"));

            Vector3 startVector = new Vector3(0.494f, 0.5f, -0.001f);
            Vector3 endVector = new Vector3(0.506f, 0.5f, -0.001f);

            //public new bool enabled = false;

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
