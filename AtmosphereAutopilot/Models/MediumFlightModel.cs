using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using UnityEngine;
using System.IO;

namespace AtmosphereAutopilot
{
	/// <summary>
	/// Class for medium-term model approximation
	/// </summary>
	class MediumFlightModel : IAutoGui, IAutoSerializable
	{
		public const int PITCH = 0;
		public const int ROLL = 1;
		public const int YAW = 2;

		Vessel vessel;

        public MediumFlightModel(Vessel v)
		{
			vessel = v;
            Deserialize();
			vessel.OnPreAutopilotUpdate += new FlightInputCallback(OnPreAutopilot);
		}

		static readonly int BUFFER_SIZE = 10;

        public CircularBuffer<double> aoa_pitch = new CircularBuffer<double>(BUFFER_SIZE, true);
        public CircularBuffer<double> aoa_yaw = new CircularBuffer<double>(BUFFER_SIZE, true);
        public CircularBuffer<double> g_force = new CircularBuffer<double>(BUFFER_SIZE, true);

		double prev_dt = 1.0;		// dt in previous call
		int stable_dt = 0;			// counts amount of stable dt intervals

		void OnPreAutopilot(FlightCtrlState state)	// update all flight characteristics
		{
			if (vessel.checkLanded())           // ground breaks the model
			{
				stable_dt = 0;
				return;
			}

			double dt = TimeWarp.fixedDeltaTime;
			check_dt(dt);
			update_buffers();
			prev_dt = dt;
		}

		void check_dt(double new_dt)
		{
			if (Math.Abs(new_dt / prev_dt - 1.0) < 0.1)
				stable_dt = Math.Min(1000, stable_dt + 1);
			else
				stable_dt = 0;
		}

		void update_buffers()
		{
            Vector3 up_srf_v = vessel.ReferenceTransform.up * Vector3.Dot(vessel.ReferenceTransform.up, vessel.srf_velocity.normalized);
            Vector3 fwd_srf_v = vessel.ReferenceTransform.forward * Vector3.Dot(vessel.ReferenceTransform.forward, vessel.srf_velocity.normalized);
            Vector3 right_srf_v = vessel.ReferenceTransform.right * Vector3.Dot(vessel.ReferenceTransform.right, vessel.srf_velocity.normalized);
            Vector3 tmpVec = up_srf_v + fwd_srf_v;
            double aoa_p = Math.Asin(Vector3.Dot(vessel.ReferenceTransform.forward.normalized, tmpVec.normalized));
            aoa_pitch.Put(aoa_p);
            tmpVec = up_srf_v + right_srf_v;
            double aoa_y = Math.Asin(Vector3.Dot(vessel.ReferenceTransform.forward.normalized, tmpVec.normalized));
            aoa_yaw.Put(aoa_y);
            g_force.Put(vessel.geeForce_immediate);
		}

        #region Serialization

        [GlobalSerializable("window_x")]
        public float WindowLeft { get { return window.xMin; } set { window.xMin = value; } }

        [GlobalSerializable("window_y")]
        public float WindowTop { get { return window.yMin; } set { window.yMin = value; } }

        [GlobalSerializable("window_width")]
        public float WindowWidth { get { return window.width; } set { window.width = value; } }

        [GlobalSerializable("window_height")]
        public float WindowHeight { get { return window.height; } set { window.height = value; } }

        public bool Deserialize()
        {
            return AutoSerialization.Deserialize(this, "MediumFlightModel",
                KSPUtil.ApplicationRootPath + "GameData/AtmosphereAutopilot/Global_settings.cfg",
                typeof(GlobalSerializable));
        }

        public void Serialize()
        {
            AutoSerialization.Serialize(this, "MediumFlightModel",
                KSPUtil.ApplicationRootPath + "GameData/AtmosphereAutopilot/Global_settings.cfg",
                typeof(GlobalSerializable));
        }

        #endregion


        #region GUI

        string module_name = "Medium-term flight model";
        int wnd_id = 8459383;
        protected bool gui_shown = false;
        bool gui_hidden = false;
        protected Rect window = new Rect(50.0f, 80.0f, 220.0f, 50.0f);

        public bool IsDrawn()
        {
            return gui_shown;
        }

        public void OnGUI()
        {
            if (!gui_shown || gui_hidden)
                return;
            window = GUILayout.Window(wnd_id, window, _drawGUI, module_name);
        }

        public bool ToggleGUI()
        {
            return gui_shown = !gui_shown;
        }

		void _drawGUI(int id)
		{
			GUILayout.BeginVertical();
			GUILayout.Label("AOA pitch = " + aoa_pitch.getLast().ToString("G8"), GUIStyles.labelStyleLeft);
            GUILayout.Label("AOA yaw = " + aoa_yaw.getLast().ToString("G8"), GUIStyles.labelStyleLeft);
            GUILayout.Label("G-force = " + g_force.getLast().ToString("G8"), GUIStyles.labelStyleLeft);
			GUILayout.EndVertical();
			GUI.DragWindow();
        }

        public void HideGUI()
        {
            gui_hidden = true;
        }

        public void ShowGUI()
        {
            gui_hidden = false;
        }

        #endregion
    }
}
