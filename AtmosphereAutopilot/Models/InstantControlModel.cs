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

		static readonly int BUFFER_SIZE = 20;

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

		#endregion

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

        Vector3 up_srf_v;		// normalized velocity, projected to vessel up direction
        Vector3 fwd_srf_v;		// normalized velocity, projected to vessel forward direction
        Vector3 right_srf_v;	// normalized velocity, projected to vessel right direction

        void update_aoa()
        {
            // thx ferram
            up_srf_v = vessel.ReferenceTransform.up * Vector3.Dot(vessel.ReferenceTransform.up, vessel.srf_velocity.normalized);
            fwd_srf_v = vessel.ReferenceTransform.forward * Vector3.Dot(vessel.ReferenceTransform.forward, vessel.srf_velocity.normalized);
            right_srf_v = vessel.ReferenceTransform.right * Vector3.Dot(vessel.ReferenceTransform.right, vessel.srf_velocity.normalized);
            
            Vector3 tmpVec = up_srf_v + fwd_srf_v;
            float aoa_p = (float)Math.Asin(Vector3.Dot(vessel.ReferenceTransform.forward.normalized, tmpVec.normalized));
            aoa[PITCH].Put(aoa_p);
            
            tmpVec = up_srf_v + right_srf_v;
            float aoa_y = (float)Math.Asin(Vector3.Dot(vessel.ReferenceTransform.right.normalized, tmpVec.normalized));
            aoa[YAW].Put(aoa_y);

            tmpVec = right_srf_v + fwd_srf_v;
            float aoa_r = (float)Math.Asin(Vector3.Dot(vessel.ReferenceTransform.forward.normalized, tmpVec.normalized));
            aoa[ROLL].Put(aoa_r);
        }

		void update_control(FlightCtrlState state)
		{
			for (int i = 0; i < 3; i++)
				input_buf[i].Put(Common.Clampf(getControlFromState(state, i), 1.0f));
		}

		public static float getControlFromState(FlightCtrlState state, int axis)
		{
			if (axis == PITCH)
				return state.pitch;
			if (axis == ROLL)
				return state.roll;
			if (axis == YAW)
				return state.yaw;
			return 0.0f;
		}

		#endregion

		#region Model_Identification

		public float[] prediction = new float[3];
        public float[] prediction_2 = new float[3];

		// Model dynamically identified parameters
		public float[] moment_aoa_k = new float[3];			// negative on stable crafts, positive or zero on unstable
		public float[] moment_aoa_b = new float[3];
		public float[] moment_input_k = new float[3];		// positive or zero

		// Regression parameters
		public float[] prediction_error = new float[3];		// Current step prediction error.

		[AutoGuiAttr("Error function memory", true)]
		[GlobalSerializable("Error function memory")]
		public int error_memory = 5;

		// Model known parameters
		[AutoGuiAttr("MOI", false)]
		public Vector3 MOI { get { return vessel.MOI; } }

		void update_model()
		{
			if (stable_dt < 4)
				return;

			// Prediction error
			for (int axis = 0; axis < 3; axis++)
			{
				prediction_error[axis] = prediction[axis] - angular_acc[axis].getLast();
			}

			// we need to initialize koeffitients on the start of flight
			if (stable_dt == 4)
			{
				for (int axis = 0; axis < 3; axis++)
				{
					moment_input_k[axis] = 1.0f;
					moment_aoa_k[axis] = 1.0f;
					moment_aoa_b[axis] = 0.0f;
					prediction[axis] = angular_acc[axis].getLast();
					prediction_2[axis] = prediction[axis];
				}
				return;
			}

			// Main model update algorithm
			if (stable_dt > error_memory + 2)
			{
				gradient_descent();
				for (int axis = 0; axis < 3; axis++)
				{
					prediction[axis] = compute_angular_acc(MOI[e_axis], moment_aoa_k[axis], moment_aoa_b[axis], aoa[axis].getLast(), 
						moment_input_k[axis], input_buf[axis].getLast());
					prediction_2[axis] = prediction[axis];
				}
			}
			else
				for (int axis = 0; axis < 3; axis++)
				{
					prediction[axis] = angular_acc[axis].getLast();
					prediction_2[axis] = prediction[axis];
				}
		}

		GradientDescent descender = new GradientDescent(3);

		[AutoGuiAttr("Descent cycles", true)]
		[GlobalSerializable("Gradient descent cycle count")]
		public int descent_iter_count = 3;

		[AutoGuiAttr("Probe delta", true)]
		[GlobalSerializable("Probe delta")]
		public float probe_delta = 1e-6f;

		[AutoGuiAttr("Descend speed", true)]
		[GlobalSerializable("Descend speed")]
		public float descent_speed = 1.0f;

		public float[] probe_deltas = new float[3];
		public float[] descent_koeff = new float[3];
		public float[][] param_arrays = { new float[3], new float[3], new float[3] };

		void gradient_descent()
		{
			for (int i = 0; i < 3; i++)
			{
				probe_deltas[i] = probe_delta;
				descent_koeff[i] = descent_speed;
			}

			for (int axis = 0; axis < 3; axis++)
			{
				e_axis = axis;
				param_arrays[axis][0] = moment_aoa_k[axis];
				param_arrays[axis][1] = moment_aoa_b[axis];
				param_arrays[axis][2] = moment_input_k[axis];
				for (int i = 0; i < descent_iter_count; i++)
					descender.apply(param_arrays[axis], error_function, probe_deltas, descent_koeff);
				moment_aoa_k[axis] = param_arrays[axis][0];
				moment_aoa_b[axis] = param_arrays[axis][1];
				moment_input_k[axis] = param_arrays[axis][2];
			}
		}

		int e_axis;
		float error_function(float[] parameters)
		{
			float error = 0.0f;
			for (int i = 0; i < error_memory; i++)
			{
				float step_accel = angular_acc[e_axis].getFromTail(i);
				float step_aoa = aoa[e_axis].getFromTail(i + 1);
				float step_input = input_buf[e_axis].getFromTail(i + 1);
				float model_accel = compute_angular_acc(MOI[e_axis], parameters[0], parameters[1], step_aoa, parameters[2], step_input);
				error += (step_accel - model_accel) * (step_accel - model_accel);
			}
			return error;
		}

		/// <summary>
		/// Model evolution function. Computes angular acceleration by Newton's law in angular form.
		/// </summary>
		/// <param name="moi">Moment of inertia</param>
		/// <param name="k_aoa">Linear air force koefficient</param>
		/// <param name="b_aoa">Air force bias</param>
		/// <param name="aoa">Angle of attack</param>
		/// <param name="k_input">Linear control input koefficient</param>
		/// <param name="b_input">Control input bias</param>
		/// <param name="input">Control input</param>
		/// <returns></returns>
		public static float compute_angular_acc(float moi, float k_aoa, float b_aoa, float aoa, float k_input, float input)
		{
			float air_moment = k_aoa * aoa + b_aoa;
			float input_moment = k_input * input;
			if (Math.Abs(moi) < 1e-3f)
				moi = 1e-3f;
			float angular_acc = (air_moment + input_moment) / moi;
			return angular_acc;
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
				GUILayout.Label(axis_names[i] + " input_authority_k = " + moment_input_k[i].ToString("G8"), GUIStyles.labelStyleLeft);
				GUILayout.Label(axis_names[i] + " aoa_authority_k = " + moment_aoa_k[i].ToString("G8"), GUIStyles.labelStyleLeft);
				GUILayout.Label(axis_names[i] + " aoa_authority_b = " + moment_aoa_b[i].ToString("G8"), GUIStyles.labelStyleLeft);
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
