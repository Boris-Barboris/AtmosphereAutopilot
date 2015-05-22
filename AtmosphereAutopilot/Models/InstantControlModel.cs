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

		static readonly int BUFFER_SIZE = 15;

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
			update_model();
			prev_dt = dt;
		}

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
		
		// Main challenge is to make control flight-model-agnostic. It means that 
		// we can't operate with basic newton's laws, because we intentionally say that forces are unknown.
		// It's very inprecise, but robust way of building a control system, and it will make the code more versatile.
		//
		// Let's formulate implicit angular acceleration evolution model:
		// a(t+dt) = G(X, c(t)), where
		// a - angular acceleration
        // t - current time
        // dt - physics time step
        // G - physics function, part of the game engine, wich does the math of applying forces. It takes following arguments:
        //   X - unknown vector of internal physics variables (angular positions, craft aerodynamics profile, altitude, speed...),
        //   c(t) - axis control value, being set by user at the moment t. It will be used as constant value by G
        //     during the "t -> t + dt" evolution.
        //
        // We don't want to know anything about G and X, so we only have a and c.
        // Next step - assumptions.
        // 1). da/dc > 0 If we pitch up a little more, we will actually pitch up harder then we would do without the increase in control.
        // 2). If we're in stable flight (angular positions are constant over some period of time), X changes slowly, in the matter of hundreds of
        //    physics steps. Basicly it means that in stabilized leveled flight fluctuations in c are more important than misc unknown factors evolution.
        //
        // 

        public float[] prediction = new float[3];
        public float[] prediction_2 = new float[3];
        public float[] linear_authority = new float[3];

        [AutoGuiAttr("Authority blending", true)]
        [GlobalSerializable("Authority blending")]
        float authority_blending = 5.0f;

        [AutoGuiAttr("Minimal control change", true)]
        [GlobalSerializable("Minimal control change")]
        float min_dcontroll = 0.04f;

		void update_model()
		{
			if (stable_dt < 3)		// minimal number of physics frames for adequate estimation
				return;

            // update authority
            for (int i = 0; i < 3; i++)
            {
                float diff = angular_acc[i].getLast() - prediction[i];
                float cntrl_diff = input_buf[i].getFromTail(1) - input_buf[i].getFromTail(2);
                if (Math.Abs(cntrl_diff) < min_dcontroll)
                    continue;
                float authority = diff / cntrl_diff / TimeWarp.fixedDeltaTime;
                if (authority > 0.0f)
                    if (linear_authority[i] != 0.0f)
                        linear_authority[i] = (linear_authority[i] * (authority_blending - 1.0f) + authority) / authority_blending;
                    else
                        linear_authority[i] = authority;
            }

            // update predictions
            for (int i = 0; i < 3; i++)
            {
                float acc = angular_acc[i].getLast();               // current angular acceleration
                float diff = acc - angular_acc[i].getFromTail(1);   // it's diffirential

                //
                // First prediction. Is estimating next physics step.
                //

                // find a situation in the past closest to current one
                int closest_index = 1;
                float min_diff = float.MaxValue;
                for (int j = 1; j < stable_dt && j < BUFFER_SIZE - 1; j++)
                {
                    float past_point = angular_acc[i].getFromTail(j);
                    float likeness = Math.Abs(acc - past_point);
                    float past_diff = past_point - angular_acc[i].getFromTail(j + 1);
                    if (likeness < min_diff && past_diff * diff > 0.0f)
                    {
                        min_diff = likeness;
                        closest_index = j;
                    }
                }

                if (closest_index == 1)
                {
                    // nothing in experience, or it's the last node. Let's just extrapolate
                    prediction[i] = acc + 0.5f * diff;
                }
                else
                {
                    float experience_diff = angular_acc[i].getFromTail(closest_index - 1) - angular_acc[i].getFromTail(closest_index);
                    float predicted_diff = (input_buf[i].getLast() - input_buf[i].getFromTail(closest_index)) * linear_authority[i] * TimeWarp.fixedDeltaTime;
                    prediction[i] = acc + experience_diff + predicted_diff;
                }

                //
                // Second prediction. Is estimating angular acceleration 2 steps ahead
                //

                acc = prediction[i];
                diff = acc - angular_acc[i].getLast();

                closest_index = 0;
                min_diff = float.MaxValue;
                for (int j = 1; j < stable_dt && j < BUFFER_SIZE - 1; j++)
                {
                    float past_point = angular_acc[i].getFromTail(j);
                    float likeness = Math.Abs(acc - past_point);
                    float past_diff = past_point - angular_acc[i].getFromTail(j + 1);
                    if (likeness < min_diff && past_diff * diff > 0.0f)
                    {
                        min_diff = likeness;
                        closest_index = j;
                    }
                }

                if (closest_index == 0)
                {
                    // nothing in experience, or it's the last node. Let's just extrapolate
                    prediction_2[i] = acc + 0.5f * diff;
                }
                else
                {
                    float experience_diff = angular_acc[i].getFromTail(closest_index - 1) - angular_acc[i].getFromTail(closest_index);
                    float predicted_diff = (input_buf[i].getLast() - input_buf[i].getFromTail(closest_index)) * linear_authority[i] * TimeWarp.fixedDeltaTime;
                    prediction_2[i] = acc + experience_diff + predicted_diff;
                }
            }
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
                GUILayout.Label(axis_names[i] + " ang acc = " + angular_acc[i].getLast().ToString("G8"), GUIStyles.labelStyleLeft);
                GUILayout.Label(axis_names[i] + " linear_authority = " + linear_authority[i].ToString("G8"), GUIStyles.labelStyleLeft);
                GUILayout.Label(axis_names[i] + " AoA = " + linear_authority[i].ToString("G8"), GUIStyles.labelStyleLeft);
				GUILayout.Space(5);
			}
            AutoGUI.AutoDrawObject(this);
			GUILayout.EndVertical();
			GUI.DragWindow();
        }

        #endregion
    }
}
