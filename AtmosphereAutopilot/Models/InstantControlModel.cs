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
	/// Short-term motion model. Is responsible for angular speed, angular acceleration, control signal
	/// history storage. Executes analysis of pitch, roll and yaw evolution and control authority.
	/// </summary>
	public sealed class InstantControlModel : AutopilotModule
	{
		internal InstantControlModel(Vessel v):
            base(v, 34278832, "Instant control model")
		{
			for (int i = 0; i < 3; i++)
			{
                input_buf[i] = new CircularBuffer<float>(BUFFER_SIZE, true, 0.0f);
                angular_v[i] = new CircularBuffer<float>(BUFFER_SIZE, true, 0.0f);
                angular_acc[i] = new CircularBuffer<float>(BUFFER_SIZE, true, 0.0f);
                aoa[i] = new CircularBuffer<float>(BUFFER_SIZE, true, 0.0f);
			}
			init_model_params();
		}

		protected override void OnActivate()
		{
			vessel.OnPreAutopilotUpdate += new FlightInputCallback(OnPreAutopilot);
			vessel.OnPostAutopilotUpdate += new FlightInputCallback(OnPostAutopilot);
		}

		protected override void OnDeactivate()
		{
			vessel.OnPreAutopilotUpdate -= new FlightInputCallback(OnPreAutopilot);
			vessel.OnPostAutopilotUpdate -= new FlightInputCallback(OnPostAutopilot);
			stable_dt = 0;
		}

		static readonly int BUFFER_SIZE = 30;

		#region Exports

		/// <summary>
		/// Control signal history for pitch, roll or yaw. [-1.0, 1.0].
		/// </summary>
		public CircularBuffer<float> ControlInputHistory(int axis) { return input_buf[axis]; }
        /// <summary>
        /// Control signal for pitch, roll or yaw. [-1.0, 1.0].
        /// </summary>
        public float ControlInput(int axis) { return input_buf[axis].getLast(); }

        CircularBuffer<float>[] input_buf = new CircularBuffer<float>[3];

		/// <summary>
		/// Angular velocity history for pitch, roll or yaw. Radians per second.
		/// </summary>
        public CircularBuffer<float> AngularVelHistory(int axis) { return angular_v[axis]; }
        /// <summary>
        /// Angular velocity for pitch, roll or yaw. Radians per second.
        /// </summary>
        public float AngularVel(int axis) { return angular_v[axis].getLast(); }

        CircularBuffer<float>[] angular_v = new CircularBuffer<float>[3];

		/// <summary>
        /// Angular acceleration hitory for pitch, roll or yaw. Radians per second per second.
		/// </summary>
        public CircularBuffer<float> AngularAccHistory(int axis) { return angular_acc[axis]; }
        /// <summary>
        /// Angular acceleration for pitch, roll or yaw. Radians per second per second.
        /// </summary>
        public float AngularAcc(int axis) { return angular_acc[axis].getLast(); }

        CircularBuffer<float>[] angular_acc = new CircularBuffer<float>[3];

        /// <summary>
        /// Angle of attack hitory for pitch, roll or yaw. Radians.
        /// </summary>
        public CircularBuffer<float> AoAHistory(int axis) { return aoa[axis]; }
        /// <summary>
        /// Angle of attack for pitch, roll or yaw. Radians.
        /// </summary>
        public float AoA(int axis) { return aoa[axis].getLast(); }

        CircularBuffer<float>[] aoa = new CircularBuffer<float>[3];

		/// <summary>
		/// Moment of inertia.
		/// </summary>
		[AutoGuiAttr("MOI", false)]
		public Vector3 MOI { get { return vessel.MOI; } }

		#endregion

		float prev_dt = 1.0f;		// dt in previous call
		int stable_dt = 0;			// amount of stable dt intervals

        void OnPostAutopilot(FlightCtrlState state)		// update control input
		{
			update_control(state);
		}

		void OnPreAutopilot(FlightCtrlState state)		// workhorse function
		{
			float dt = TimeWarp.fixedDeltaTime;
			check_dt(dt);
			update_velocity_acc();
            update_aoa();
			if (vessel.LandedOrSplashed)
				stable_dt = 0;
			update_model();
			prev_dt = dt;
		}

		#region Angular_Update

		void check_dt(float new_dt)	// check if dt is consistent with previous one
		{
			if (Math.Abs(new_dt / prev_dt - 1.0) < 0.1)
				stable_dt = Math.Min(1000, stable_dt + 1);	// overflow protection
			else
				stable_dt = 0;			// dt has changed
		}

		void update_velocity_acc()
		{
			for (int i = 0; i < 3; i++)
			{
				angular_v[i].Put(-vessel.angularVelocity[i]);	// update angular velocity. Minus for more meaningful
																// numbers (pitch up is positive etc.)
				if (stable_dt > 0)
					angular_acc[i].Put(
						(float)Common.derivative1_short(
							angular_v[i].getFromTail(1),
							angular_v[i].getLast(),
							prev_dt));
			}
		}

        Vector3 up_srf_v;		// velocity, projected to vessel up direction
        Vector3 fwd_srf_v;		// velocity, projected to vessel forward direction
        Vector3 right_srf_v;	// velocity, projected to vessel right direction

        void update_aoa()
        {
            // thx ferram
            up_srf_v = vessel.ReferenceTransform.up * Vector3.Dot(vessel.ReferenceTransform.up, vessel.srf_velocity);
            fwd_srf_v = vessel.ReferenceTransform.forward * Vector3.Dot(vessel.ReferenceTransform.forward, vessel.srf_velocity);
            right_srf_v = vessel.ReferenceTransform.right * Vector3.Dot(vessel.ReferenceTransform.right, vessel.srf_velocity);

			Vector3 tmpVec = up_srf_v + fwd_srf_v;
			if (tmpVec.sqrMagnitude > 1.0f)
			{
				float aoa_p = (float)Math.Asin(Common.Clampf(Vector3.Dot(vessel.ReferenceTransform.forward.normalized, tmpVec.normalized), 1.0f));
				if (Vector3.Dot(tmpVec, vessel.ReferenceTransform.up) < 0.0)
					aoa_p = (float)Math.PI - aoa_p;
				aoa[PITCH].Put(aoa_p);
			}
			else
				aoa[PITCH].Put(0.0f);
			
			tmpVec = up_srf_v + right_srf_v;
			if (tmpVec.sqrMagnitude > 1.0f)
			{
				float aoa_y = (float)Math.Asin(Common.Clampf(Vector3.Dot(vessel.ReferenceTransform.right.normalized, tmpVec.normalized), 1.0f));
				if (Vector3.Dot(tmpVec, vessel.ReferenceTransform.up) < 0.0)
					aoa_y = (float)Math.PI - aoa_y;
				aoa[YAW].Put(aoa_y);
			}
			else
				aoa[YAW].Put(0.0f);

			tmpVec = right_srf_v + fwd_srf_v;
			if (tmpVec.sqrMagnitude > 1.0f)
			{
				float aoa_r = (float)Math.Asin(Common.Clampf(Vector3.Dot(vessel.ReferenceTransform.forward.normalized, tmpVec.normalized), 1.0f));
				if (Vector3.Dot(tmpVec, vessel.ReferenceTransform.right) < 0.0)
					aoa_r = (float)Math.PI - aoa_r;
				aoa[ROLL].Put(aoa_r);
			}
			else
				aoa[ROLL].Put(0.0f);
        }

		void update_control(FlightCtrlState state)
		{
			for (int i = 0; i < 3; i++)
				input_buf[i].Put(Common.Clampf(ControlUtils.getControlFromState(state, i), 1.0f));
		}

		#endregion

		#region Model_Identification

		// Model dynamically identified parameters
		double[][] model_parameters = { new double[4], new double[4], new double[4] };

		void init_model_params()
		{
			for (int axis = 0; axis < 3; axis++)
			{
				model_parameters[axis][0] = 0.0;
				model_parameters[axis][1] = 0.0;
				model_parameters[axis][2] = 100.0;
				model_parameters[axis][3] = -10.0;
			}
		}

		/// <summary>
		/// Linear coefficient before angle of attack. Negative on stable crafts, positive or zero on unstable.
		/// </summary>
		public double moment_aoa_k(int axis) { return model_parameters[axis][0]; }

		/// <summary>
		/// Momentum bias. Sometimes craft is assimetric.
		/// </summary>
		public double moment_aoa_b(int axis) { return model_parameters[axis][1]; }

		/// <summary>
		/// Linear control authority.
		/// </summary>
		public double moment_input_k(int axis) { return model_parameters[axis][2]; }

		/// <summary>
		/// Dissipative coefficient, is responsible for rotational friction. Is multiplied on an
		/// angular velocity square. Can't be positive.
		/// </summary>
		public double moment_v_d(int axis) { return model_parameters[axis][3]; }
		

		// Regression performance
		public float[] prediction_error = new float[3];		// Current step prediction error.
		public float[] prediction = new float[3];			// prediction for next step

		/// <summary>
		/// Computes angular acceleration in scalar form around one axis. Works in atmosphere.
		/// </summary>
		/// <param name="moi">Moment of inertia</param>
		/// <param name="k_aoa">Linear air force koefficient</param>
		/// <param name="b_aoa">Air force bias</param>
		/// <param name="aoa">Angle of attack</param>
		/// <param name="k_input">Linear control input koefficient</param>
		/// <param name="b_input">Control input bias</param>
		/// <param name="input">Control input</param>
		/// <returns>Angular acceleration</returns>
		public static double model_angular_acc(double moi, double k_aoa, double b_aoa, double aoa,
			double k_input, double input, double v_d, double v)
		{
			double air_moment = k_aoa * aoa + b_aoa;
			double input_moment = k_input * input;
			double friction_moment = v_d * v * v;
			if (Math.Abs(moi) < 1e-3)
				moi = 1e-3;
			double angular_acc = (air_moment + input_moment + friction_moment) / moi;
			return angular_acc;
		}

		int call_counter = 0;
		double model_angular_acc(int axis, int past, double[] parameters)
		{
			call_counter++;
			return model_angular_acc(MOI[axis], parameters[0], parameters[1], aoa[axis].getFromTail(past+1),
				parameters[2], input_buf[axis].getFromTail(past+1), parameters[3], angular_v[axis].getFromTail(past+1));
		}

		double error_function(int axis, int frame_size, double[] parameters)
		{
			double meansqr = 0.0;
			for (int i = 0; i < frame_size; i++)
			{
				double err = model_angular_acc(axis, i, parameters) - angular_acc[axis].getFromTail(i);
				err *= err;
				meansqr += err;
			}
			return meansqr / (double)frame_size;
		}

		GradientDescend optimizer = new GradientDescend(4);

		[AutoGuiAttr("frame size", true)]
		[GlobalSerializable("frame_size")]
		int frame_size = 7;

		[AutoGuiAttr("max calls", true)]
		[GlobalSerializable("max_calls")]
		int Max_calls { get { return optimizer.max_func_calls; } set { optimizer.max_func_calls = value; } }

		[AutoGuiAttr("descend_k", true, "G6")]
		[GlobalSerializable("descend_k")]
		double Descend_k { get { return optimizer.descend_k; } set { optimizer.descend_k = value; } }

		[AutoGuiAttr("miss_k", true, "G6")]
		[GlobalSerializable("miss_k")]
		double Miss_k { get { return optimizer.descend_miss_k; } set { optimizer.descend_miss_k = value; } }

		void update_model()
		{
			// Prediction error
			for (int axis = 0; axis < 3; axis++)
			{
				prediction_error[axis] = prediction[axis] - angular_acc[axis].getLast();
			}

			int avail_frame = stable_dt - 2;
			if (avail_frame > 1)
			{
				// we have data to proceed
				for (int axis = 0; axis < 3; axis++)
				{
					call_counter = 0;
					// apply gradient descend
					optimizer.apply(model_parameters[axis], 
						(p) => { return error_function(axis, Math.Min(avail_frame, frame_size), p); }, ref call_counter);
					// make predictions for next cycle
					prediction[axis] = (float)model_angular_acc(axis, -1, model_parameters[axis]);
				}
			}
		}

		#endregion

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
        static readonly float rad_to_degree = (float)(180.0 / Math.PI);

		protected override void _drawGUI(int id)
		{
			GUILayout.BeginVertical();
			for (int i = 0; i < 3; i++)
			{
				GUILayout.Label(axis_names[i] + " ang vel = " + angular_v[i].getLast().ToString("G8"), GUIStyles.labelStyleLeft);
                GUILayout.Label(axis_names[i] + " ang acc = " + angular_acc[i].getLast().ToString("G8"), GUIStyles.labelStyleLeft);
                GUILayout.Label(axis_names[i] + " AoA = " + (aoa[i].getLast() * rad_to_degree).ToString("G8"), GUIStyles.labelStyleLeft);
				GUILayout.Label(axis_names[i] + " aoa_k = " + moment_aoa_k(i).ToString("G8"), GUIStyles.labelStyleLeft);
				GUILayout.Label(axis_names[i] + " aoa_b = " + moment_aoa_b(i).ToString("G8"), GUIStyles.labelStyleLeft);
				GUILayout.Label(axis_names[i] + " v_d = " + moment_v_d(i).ToString("G8"), GUIStyles.labelStyleLeft);
				GUILayout.Label(axis_names[i] + " input_k = " + moment_input_k(i).ToString("G8"), GUIStyles.labelStyleLeft);
				GUILayout.Label(axis_names[i] + " prediction_error = " + prediction_error[i].ToString("G8"), GUIStyles.labelStyleLeft);
				GUILayout.Space(5);
			}
            AutoGUI.AutoDrawObject(this);
			GUILayout.EndVertical();
			GUI.DragWindow();
        }

        #endregion
    }
}
