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

    /// <summary>
    /// Short-term motion model. Is responsible for angular velocity, angular acceleration, control signal and 
    /// angle of attack evaluation and storage. Executes analysis of pitch, roll and yaw evolution and control authority.
    /// </summary>
    public sealed partial class FlightModel : AutopilotModule
    {
        internal FlightModel(Vessel v) :
            base(v, 34278832, "Flight model")
        {
            for (int i = 0; i < 3; i++)
            {
                input_buf[i] = new CircularBufferAA<float>(BUFFER_SIZE, true);
                csurf_buf[i] = new CircularBufferAA<float>(BUFFER_SIZE, true);
                gimbal_buf[i] = new CircularBufferAA<float>(BUFFER_SIZE, true);
                angular_v_buf[i] = new CircularBufferAA<float>(BUFFER_SIZE, true);
                angular_acc_buf[i] = new CircularBufferAA<double>(BUFFER_SIZE, true);
                aoa_buf[i] = new CircularBufferAA<float>(BUFFER_SIZE, true);
            }
            initialize_lin_tainers();
            //integrator = vessel.GetComponent<FlightIntegrator>();
            window.width = 240.0f;  // some vector components need love
        }

        //FlightIntegrator integrator;

        protected override void OnActivate()
        {
            sequential_dt = false;
            vessel.OnPreAutopilotUpdate += new FlightInputCallback(OnPreAutopilot);
            vessel.OnPostAutopilotUpdate += new FlightInputCallback(OnPostAutopilot);
            AtmosphereAutopilot.Instance.BackgroundThread.add_func(train_pitch_ann);
            AtmosphereAutopilot.Instance.BackgroundThread.add_func(train_roll_ann);
            AtmosphereAutopilot.Instance.BackgroundThread.add_func(train_yaw_ann);
            AtmosphereAutopilot.Instance.BackgroundThread.add_func(train_pitch_lift);
            AtmosphereAutopilot.Instance.BackgroundThread.add_func(train_yaw_lift);
            AtmosphereAutopilot.Instance.BackgroundThread.Start();
        }

        protected override void OnDeactivate()
        {
            vessel.OnPreAutopilotUpdate -= new FlightInputCallback(OnPreAutopilot);
            vessel.OnPostAutopilotUpdate -= new FlightInputCallback(OnPostAutopilot);
            AtmosphereAutopilot.Instance.BackgroundThread.remove_func(train_pitch_ann);
            AtmosphereAutopilot.Instance.BackgroundThread.remove_func(train_roll_ann);
            AtmosphereAutopilot.Instance.BackgroundThread.remove_func(train_yaw_ann);
            AtmosphereAutopilot.Instance.BackgroundThread.remove_func(train_pitch_lift);
            AtmosphereAutopilot.Instance.BackgroundThread.remove_func(train_yaw_lift);
            return_gimbals();
            sequential_dt = false;
            moments_cycle_counter = 0;
        }

        static readonly int BUFFER_SIZE = 8;



        #region Buffers

        CircularBufferAA<float>[] input_buf = new CircularBufferAA<float>[3];

        CircularBufferAA<float>[] csurf_buf = new CircularBufferAA<float>[3];   // True control surface action values

        CircularBufferAA<float>[] gimbal_buf = new CircularBufferAA<float>[3];   // True gimbal action values

        CircularBufferAA<float>[] angular_v_buf = new CircularBufferAA<float>[3];

        CircularBufferAA<double>[] angular_acc_buf = new CircularBufferAA<double>[3];

        CircularBufferAA<float>[] aoa_buf = new CircularBufferAA<float>[3];

        #endregion



        #region BufferExports

        /// <summary>
        /// Control signal history for pitch, roll or yaw. [-1.0, 1.0].
        /// </summary>
        public CircularBufferAA<float> ControlInputHistory(int axis) { return input_buf[axis]; }
        /// <summary>
        /// Control signal for pitch, roll or yaw. [-1.0, 1.0].
        /// </summary>
        public float ControlInput(int axis) { return input_buf[axis].getLast(); }

        /// <summary>
        /// Lagged control surface position history for pitch, roll or yaw. [-1.0, 1.0].
        /// </summary>
        public CircularBufferAA<float> ControlSurfPosHistory(int axis) { return csurf_buf[axis]; }
        /// <summary>
        /// Lagged control surface position for pitch, roll or yaw. [-1.0, 1.0].
        /// </summary>
        public float ControlSurfPos(int axis) { return csurf_buf[axis].getLast(); }

        /// <summary>
        /// Lagged gimbal position history for pitch, roll or yaw. [-1.0, 1.0].
        /// </summary>
        public CircularBufferAA<float> GimbalPosHistory(int axis) { return gimbal_buf[axis]; }
        /// <summary>
        /// Lagged gimbal position for pitch, roll or yaw. [-1.0, 1.0].
        /// </summary>
        public float GimbalPos(int axis) { return gimbal_buf[axis].getLast(); }

        /// <summary>
        /// Angular velocity history for pitch, roll or yaw. Radians per second.
        /// </summary>
        public CircularBufferAA<float> AngularVelHistory(int axis) { return angular_v_buf[axis]; }
        /// <summary>
        /// Angular velocity for pitch, roll or yaw. Radians per second.
        /// </summary>
        public float AngularVel(int axis) { return angular_v_buf[axis].getLast(); }

        /// <summary>
        /// Angular acceleration hitory for pitch, roll or yaw. Radians per second per second.
        /// </summary>
        public CircularBufferAA<double> AngularAccHistory(int axis) { return angular_acc_buf[axis]; }
        /// <summary>
        /// Angular acceleration for pitch, roll or yaw. Radians per second per second.
        /// </summary>
        public double AngularAcc(int axis) { return angular_acc_buf[axis].getLast(); }

        /// <summary>
        /// Angle of attack hitory for pitch, roll or yaw. Radians.
        /// </summary>
        public CircularBufferAA<float> AoAHistory(int axis) { return aoa_buf[axis]; }
        /// <summary>
        /// Angle of attack for pitch, roll or yaw. Radians.
        /// </summary>
        public float AoA(int axis) { return aoa_buf[axis].getLast(); }

        #endregion



        void OnPostAutopilot(FlightCtrlState state)     // update control input
        {
            update_control(state);
            if (!vessel.LandedOrSplashed)
                sequential_dt = true;
            postupdate_dynamics();
            postupdate_engine_balancing(state);
        }

        internal bool sequential_dt = false;
        uint reference_transform_id = uint.MaxValue;

        void OnPreAutopilot(FlightCtrlState state)      // workhorse function
        {
            if (reference_transform_id != vessel.referenceTransformId)
                sequential_dt = false;
            reference_transform_id = vessel.referenceTransformId;

            update_moments();
            update_velocity_acc();
            update_aoa();
            update_engine_moments();
            get_gimbal_authority();
            if (moments_cycle_counter == 1)
                init_engine_balancing();
            update_engine_balancing();
            update_dynamics();

            if (sequential_dt)
            {
                if (angular_acc_buf[0].Size > 0 && !vessel.LandedOrSplashed)
                {
                    update_model_acc();
                    update_training_inputs();
                    update_cpu();
                    update_pitch_rot_model();
                    update_roll_rot_model();
                    update_yaw_rot_model();
                }
            }
        }

        #region Serialization

        public override bool Deserialize()
        {
            bool res = base.Deserialize();
            initialize_trainer_windows();
            return res;
        }

        #endregion

        #region GUI

        static readonly string[] axis_names = { "pitch", "roll", "yaw" };
        const float rad2degree = (float)(180.0 / Math.PI);

        protected override void _drawGUI(int id)
        {
            close_button();
            GUILayout.BeginVertical();
            for (int i = 0; i < 3; i++)
            {
                GUILayout.Label("=======" + axis_names[i] + "=======");
                GUILayout.Label("ang vel = " + angular_v_buf[i].getLast().ToString("G8"), GUIStyles.labelStyleLeft);
                GUILayout.Label("ang acc = " + angular_acc_buf[i].getLast().ToString("G8"), GUIStyles.labelStyleLeft);
                GUILayout.Label("AoA = " + (aoa_buf[i].getLast() * rad2degree).ToString("G8"), GUIStyles.labelStyleLeft);
            }
            AutoGUI.AutoDrawObject(this);
            GUILayout.EndVertical();
            GUI.DragWindow();
        }

        OnlineLinTrainerWindow pitch_lin_wnd, roll_lin_wnd, yaw_lin_wnd,
            lift_lin_wnd, slide_lin_wnd;
        OnlineLinTrainerWindow[] trainer_windows = new OnlineLinTrainerWindow[5];
        //int shown_trainer = -1;

        void initialize_trainer_windows()
        {
            Rect lin_wnd_rect = window;
            lin_wnd_rect.xMin = window.xMin - 190.0f;
            lin_wnd_rect.xMax = lin_wnd_rect.xMin + 190.0f;
            trainer_windows[0] = pitch_lin_wnd = new OnlineLinTrainerWindow(pitch_trainer, "pitch trainer", 908999, lin_wnd_rect);
            trainer_windows[1] = roll_lin_wnd = new OnlineLinTrainerWindow(roll_trainer, "roll trainer", 908998, lin_wnd_rect);
            trainer_windows[2] = yaw_lin_wnd = new OnlineLinTrainerWindow(yaw_trainer, "yaw trainer", 908997, lin_wnd_rect);
            trainer_windows[3] = lift_lin_wnd = new OnlineLinTrainerWindow(pitch_lift_trainer, "lift trainer", 908996, lin_wnd_rect);
            trainer_windows[4] = slide_lin_wnd = new OnlineLinTrainerWindow(yaw_lift_trainer, "slide trainer", 908995, lin_wnd_rect);
        }

        void unshow_all_trainer_windows()
        {
            foreach (var tw in trainer_windows)
                tw.UnShowGUI();
        }

        [AutoGuiAttr("pitch trainer", true)]
        bool show_pitch_lin_wnd
        {
            get { return pitch_lin_wnd.IsShown(); }
            set { if (value) { unshow_all_trainer_windows(); pitch_lin_wnd.ShowGUI(); } else pitch_lin_wnd.UnShowGUI(); }
        }

        [AutoGuiAttr("roll trainer", true)]
        bool show_roll_lin_wnd
        {
            get { return roll_lin_wnd.IsShown(); }
            set { if (value) { unshow_all_trainer_windows(); roll_lin_wnd.ShowGUI(); } else roll_lin_wnd.UnShowGUI(); }
        }

        [AutoGuiAttr("yaw trainer", true)]
        bool show_yaw_lin_wnd
        {
            get { return yaw_lin_wnd.IsShown(); }
            set { if (value) { unshow_all_trainer_windows(); yaw_lin_wnd.ShowGUI(); } else yaw_lin_wnd.UnShowGUI(); }
        }

        [AutoGuiAttr("lift trainer", true)]
        bool show_lift_lin_wnd
        {
            get { return lift_lin_wnd.IsShown(); }
            set { if (value) { unshow_all_trainer_windows(); lift_lin_wnd.ShowGUI(); } else lift_lin_wnd.UnShowGUI(); }
        }

        [AutoGuiAttr("slide trainer", true)]
        bool show_slide_lin_wnd
        {
            get { return slide_lin_wnd.IsShown(); }
            set { if (value) { unshow_all_trainer_windows(); slide_lin_wnd.ShowGUI(); } else slide_lin_wnd.UnShowGUI(); }
        }

        protected override void OnGUICustom()
        {
            foreach (var trainer in trainer_windows)
                trainer.OnGUI();
        }

        #endregion
    }
}
