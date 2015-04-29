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
	/// Short-motion model. Is responsible for angular speed, angular acceleration, control signal
	/// history storage. Executes analysis of pitch, roll and yaw evolution and control authority.
	/// </summary>
	public sealed class InstantControlModel : AutopilotModule
	{
		internal InstantControlModel(Vessel v):
            base(v, 34278832, "Instant control model")
		{
			for (int i = 0; i < 3; i++)
			{
				input_buf[i] = new CircularBuffer<double>(BUFFER_SIZE, true, 0.0);
                angular_v[i] = new CircularBuffer<double>(BUFFER_SIZE, true, 0.0);
                angular_dv[i] = new CircularBuffer<double>(BUFFER_SIZE, true, 0.0);
			}
		}

		protected override void OnActivate()
		{
			vessel.OnPreAutopilotUpdate += new FlightInputCallback(OnPreAutopilot);
			vessel.OnPostAutopilotUpdate += new FlightInputCallback(OnPostAutopilot);
		}

		protected override void OnDeactivate()
		{
			vessel.OnPreAutopilotUpdate += new FlightInputCallback(OnPreAutopilot);
			vessel.OnPostAutopilotUpdate += new FlightInputCallback(OnPostAutopilot);
			stable_dt = 0;
		}

		static readonly int BUFFER_SIZE = 15;
        static readonly int MISTAKE_BUF_SIZE = 15;

		/// <summary>
		/// Control signal history for pitch, roll or yaw. [-1.0, 1.0].
		/// </summary>
		public CircularBuffer<double> InputHistory(int axis) { return input_buf[axis]; }
		internal CircularBuffer<double>[] input_buf = new CircularBuffer<double>[3];

		/// <summary>
		/// Angular velocity history for pitch, roll or yaw. Radians/second.
		/// </summary>
		public CircularBuffer<double> AngularVelHistory(int axis) { return angular_v[axis]; }
		internal CircularBuffer<double>[] angular_v = new CircularBuffer<double>[3];

		/// <summary>
		/// Angular acceleration hitory for pitch, roll or yaw. Caution: extremely noisy! Radians/second^2.
		/// </summary>
		public CircularBuffer<double> AngularAccHistory(int axis) { return angular_dv[axis]; }
		internal CircularBuffer<double>[] angular_dv = new CircularBuffer<double>[3];

		float prev_dt = 1.0f;		// dt in previous call
		int stable_dt = 0;			// amount of stable dt intervals

        void OnPostAutopilot(FlightCtrlState state)		// update control input
		{
			update_control(state);
		}

		void OnPreAutopilot(FlightCtrlState state)		// update all flight characteristics
		{
			float dt = TimeWarp.fixedDeltaTime;
			check_dt(dt);
			update_buffers();
			update_dv_model();
			prev_dt = dt;
		}

		void check_dt(float new_dt)	// check if dt is consistent with previous one
		{
			if (Math.Abs(new_dt / prev_dt - 1.0) < 0.1)
				stable_dt = Math.Min(1000, stable_dt + 1);	// overflow protection
			else
				stable_dt = 0;			// dt has changed
		}

		void update_buffers()
		{
			for (int i = 0; i < 3; i++)
			{
				angular_v[i].Put(-vessel.angularVelocity[i]);	// update angular velocity. Minus for more meaningful
																// numbers. Squad uses RHS coord system. Can't deal with it.
				if (stable_dt > 2)
					angular_dv[i].Put(
						Common.derivative1_short(
							angular_v[i].getFromTail(1),
							angular_v[i].getLast(),
							prev_dt));
			}
		}

		void update_control(FlightCtrlState state)
		{
			for (int i = 0; i < 3; i++)
				input_buf[i].Put(Common.Clamp(getControlFromState(state, i), 1.0));
		}

		double getControlFromState(FlightCtrlState state, int control)
		{
			if (control == PITCH)
				return state.pitch;
			if (control == ROLL)
				return state.roll;
			if (control == YAW)
				return state.yaw;
			return 0.0;
		}
		
		// Main challenge is to make control flight model-agnostic. It means that 
		// we can't operate with basic newton's laws, because we intentionally say that forces are unknown.
		// It will make the code more versatile.
		//
		// Let's formulate implicit angular acceleration evolution model:
		// a(t+dt) = G(X, t, c(t))
		// 

		void update_dv_model()
		{
			if (stable_dt < 10)		// minimal number of physics frames for adequate estimation
				return;

		}



        #region Serialization

        public override bool Deserialize()
        {
            return AutoSerialization.Deserialize(this, "InstantControlModel",
                KSPUtil.ApplicationRootPath + "GameData/AtmosphereAutopilot/Global_settings.cfg",
                typeof(GlobalSerializable));
        }

        public override void Serialize()
        {
            AutoSerialization.Serialize(this, "InstantControlModel",
                KSPUtil.ApplicationRootPath + "GameData/AtmosphereAutopilot/Global_settings.cfg",
                typeof(GlobalSerializable));
        }

        #endregion


        #region GUI

		static readonly string[] axis_names = { "pitch", "roll", "yaw" };

		protected override void _drawGUI(int id)
		{
			GUILayout.BeginVertical();
			for (int i = 0; i < 3; i++)
			{
				GUILayout.Label(axis_names[i] + " ang vel = " + angular_v[i].getLast().ToString("G8"), GUIStyles.labelStyleLeft);
                GUILayout.Label(axis_names[i] + " ang vel d1 = " + angular_dv[i].getLast().ToString("G8"), GUIStyles.labelStyleLeft);
				GUILayout.Space(5);
			}
            AutoGUI.AutoDrawObject(this);
			GUILayout.EndVertical();
			GUI.DragWindow();
        }

        #endregion
    }
}
