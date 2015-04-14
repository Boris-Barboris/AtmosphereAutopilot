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
	/// history storage. Executes analysis of pitch, roll and yaw control authority.
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
                angular_dv_central[i] = new CircularBuffer<double>(BUFFER_SIZE, true, 0.0);
                k_dv_control[i] = new CircularBuffer<double>(10, true, min_authority_dv);
                dv_mistake[i] = new CircularBuffer<double>(MISTAKE_BUF_SIZE, true, 0.0);
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
        
		// Filtered angular acceleration
		internal CircularBuffer<double>[] angular_dv_central = new CircularBuffer<double>[3];

		double prev_dt = 1.0;		// dt in previous call
		int stable_dt = 0;			// amount of stable dt intervals

        void OnPostAutopilot(FlightCtrlState state)		// update control input
		{
			update_control(state);
		}

		void OnPreAutopilot(FlightCtrlState state)		// update all flight characteristics
		{
			double dt = TimeWarp.fixedDeltaTime;
			check_dt(dt);
			update_buffers();
			update_dv_model();
			prev_dt = dt;
		}

		void check_dt(double new_dt)	// check if dt is consistent with previous one
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
				angular_v[i].Put(-vessel.angularVelocity[i]);	// update angular velocity
				if (stable_dt > 2)
					angular_dv[i].Put(
						Common.derivative1_short(
							angular_v[i].getFromTail(1),
							angular_v[i].getFromTail(0),
							prev_dt));
                if (stable_dt >= 7)
                    angular_dv_central[i].Put(smooth_derivative_central(prev_dt, i));
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

        /// <summary>
        /// Smooth central noise-robust differentiator, author - Pavel Holoborodko. Based on Savitzky–Golay filter.
        /// http://www.holoborodko.com/pavel/wp-content/uploads/OneSidedNoiseRobustDifferentiators.pdf
        /// </summary>
        double smooth_derivative_central(double dt, int axis)
        {
            double result =
                -1.0 * angular_v[axis].getFromTail(6) +
                -4.0 * angular_v[axis].getFromTail(5) +
                -5.0 * angular_v[axis].getFromTail(4) +
                5.0 * angular_v[axis].getFromTail(2) +
                4.0 * angular_v[axis].getFromTail(1) +
                1.0 * angular_v[axis].getFromTail(0);
            result /= 32.0 * dt;
            return result;
        }

		//
		// Short-time model for angular acceleration
		//
        // Let's assume that at some time interval dt control signal has changed by "d_control", 
		// and angular acceleration has changed by "d_acc".
		// d_acc/d_control - instant control authority.
		// Due to huge amount of high-frequency noise in acceleration data we'll have to use
		// old filtered acceleration data for authority analysis. If instant authority is consistent
		// with common sense (positive and larger than some small value), it will
		// be stored at buffer - k_dv_control. Average of this buffer will be used as authority.
		// Diffirence between filtered and raw acceleration will be stored in dv_mistake.

		internal CircularBuffer<double>[] k_dv_control = new CircularBuffer<double>[3];
        internal CircularBuffer<double>[] dv_mistake = new CircularBuffer<double>[3];
        internal double[] dv_avg_mistake = new double[3];

		public void update_dv_model()
		{
			if (stable_dt < 10)
				return;

			for (int i = 0; i < 3; i++)
			{
                //double d_control = input_buf[i].getFromTail(3) - input_buf[i].getFromTail(4);

                //if (Math.Abs(d_control) > min_d_short_control)        // if d_control is substantial
                //{
                //    // get control authority in acceleration
                //    double d1dv = Common.derivative1_short(angular_dv_central[i].getFromTail(2),
                //        angular_dv_central[i].getFromTail(1), prev_dt);
                //    double d2dv = Common.derivative2(angular_dv_central[i].getFromTail(3),
                //        angular_dv_central[i].getFromTail(2),  angular_dv_central[i].getFromTail(1), prev_dt);
                //    double extrapolated_dv = Common.extrapolate(angular_dv_central[i].getFromTail(1),
                //        d1dv, d2dv, prev_dt);
                //    double control_authority_dv = (angular_dv_central[i].getLast() - extrapolated_dv) / d_control;
                //    if (control_authority_dv > min_authority_dv)
                //        k_dv_control[i].Put(control_authority_dv);
                //    else
                //        k_dv_control[i].Put(min_authority_dv);
                //}

                //double d_control = input_buf[i].getFromTail(0) - input_buf[i].getFromTail(1);

                //if (Math.Abs(d_control) > min_d_short_control)        // if d_control is substantial
                //{
                //    // get control authority in acceleration
                //    double d1dv = Common.derivative1_short(angular_dv[i].getFromTail(2),
                //        angular_dv[i].getFromTail(1), prev_dt);
                //    double d2dv = Common.derivative2(angular_dv[i].getFromTail(3),
                //        angular_dv[i].getFromTail(2), angular_dv[i].getFromTail(1), prev_dt);
                //    double extrapolated_dv = Common.extrapolate(angular_dv[i].getFromTail(1),
                //        d1dv, d2dv, prev_dt);
                //    double control_authority_dv = (angular_dv[i].getLast() - extrapolated_dv) / d_control;
                //    if (control_authority_dv > min_authority_dv)
                //        k_dv_control[i].Put(control_authority_dv);
                //}

                //double d_control = input_buf[i].getFromTail(3) - input_buf[i].getFromTail(4);

                //if (Math.Abs(d_control) > min_d_short_control)        // if d_control is substantial
                //{
                //    // get instant control authority
                //    double control_authority_dv = 
                //        (angular_dv_central[i].getLast() - angular_dv_central[i].getFromTail(1)) / d_control;
                //    if (control_authority_dv > min_authority_dv)
                //        k_dv_control[i].Put(control_authority_dv);
                //}

                double d_control = input_buf[i].getFromTail(1) - input_buf[i].getFromTail(2);

                if (Math.Abs(d_control) > min_d_short_control)        // if d_control is substantial
                {
                    // get instant control authority
                    double control_authority_dv =
                        (angular_dv[i].getLast() - angular_dv[i].getFromTail(1)) / d_control;
                    if (control_authority_dv > min_authority_dv)
                        k_dv_control[i].Put(control_authority_dv);
                }

				// update acceleration noise history
                double cur_mistake = Math.Abs(angular_dv_central[i].getLast() - angular_dv[i].getFromTail(3));
                dv_mistake[i].Put(cur_mistake);
                if (dv_mistake.Length < MISTAKE_BUF_SIZE)
                    dv_avg_mistake[i] = (dv_avg_mistake[i] * dv_mistake.Length + cur_mistake) / (dv_mistake.Length + 1);
                else
                    dv_avg_mistake[i] = (dv_avg_mistake[i] * mistake_averaging_gain + cur_mistake) / (mistake_averaging_gain + 1);
			}
		}

        [AutoGuiAttr("min_d_short_control", true, "G6")]
        [GlobalSerializable("min_d_short_control")]
        internal double min_d_short_control = 0.05;

        [AutoGuiAttr("min_authority_dv", true, "G6")]
        [GlobalSerializable("min_authority_dv")]
        internal double min_authority_dv = 0.1;

        [AutoGuiAttr("mistake_averaging_gain", true, "G6")]
        [GlobalSerializable("mistake_averaging_gain")]
        internal int mistake_averaging_gain = 1000;

		/// <summary>
		/// Get averaged control authority for specified rotation axis.
		/// </summary>
        public double getDvAuthority(int axis)
        {
            if (k_dv_control[axis].Size > 0)
                return k_dv_control[axis].Average();
            else
                return 1.0;
        }

		/// <summary>
		/// Get last instant control authority for specified rotation axis.
		/// </summary>
		public double getDvAuthorityInstant(int axis)
        {
            return k_dv_control[axis].getLast();
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
                GUILayout.Label(axis_names[i] + " K dv = " + k_dv_control[i].getLast().ToString("G8"), GUIStyles.labelStyleLeft);
				GUILayout.Space(5);
			}
            AutoGUI.AutoDrawObject(this);
			GUILayout.EndVertical();
			GUI.DragWindow();
        }

        #endregion
    }
}
