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
	/// Class for short-motion model approximation
	/// </summary>
	class InstantControlModel : IAutoGui
	{
		public static readonly int PITCH = 0;
		public static readonly int ROLL = 1;
		public static readonly int YAW = 2;

		Vessel vessel;

		public InstantControlModel(Vessel v)
		{
			vessel = v;
			for (int i = 0; i < 3; i++)
			{
				input_buf[i] = new CircularBuffer<double>(BUFFER_SIZE, true);
				angular_v[i] = new CircularBuffer<double>(BUFFER_SIZE, true);
				angular_dv[i] = new CircularBuffer<double>(BUFFER_SIZE, true);
				angular_d2v[i] = new CircularBuffer<double>(BUFFER_SIZE, true);
			}
            vessel.OnFlyByWire += new FlightInputCallback(OnFlyByWire);
		}

		static readonly int BUFFER_SIZE = 50;

		public CircularBuffer<double>[] input_buf = new CircularBuffer<double>[3];	// control input value, forward from others on 1 dt
		public CircularBuffer<double>[] angular_v = new CircularBuffer<double>[3];	// angular v
		public CircularBuffer<double>[] angular_dv = new CircularBuffer<double>[3];	// dv/dt
		public CircularBuffer<double>[] angular_d2v = new CircularBuffer<double>[3];// d2v/dt2

		double prev_dt = 1.0;		// dt in previous call
		int stable_dt = 0;			// counts amount of stable dt intervals

        void OnFlyByWire(FlightCtrlState state)
		{
			if (vessel.checkLanded())           // ground breaks the model
			{
				stable_dt = 0;
				return;
			}
			
			double dt = TimeWarp.fixedDeltaTime;
			check_dt(dt);
			update_buffers(state);
			update_model();

			prev_dt = dt;
		}

		void check_dt(double new_dt)
		{
			if (Math.Abs(new_dt / prev_dt - 1.0) < 0.1)
				stable_dt = Math.Min(1000, stable_dt + 1);
			else
				stable_dt = 0;
		}

		void update_buffers(FlightCtrlState state)
		{
			for (int i = 0; i < 3; i++)
			{
				input_buf[i].Put(getControlFromState(state, i));
				angular_v[i].Put(vessel.angularVelocity[i]);
				if (stable_dt >= 1)
					angular_dv[i].Put(
                        derivative1_short(
							angular_v[i].getFromTail(1),
							angular_v[i].getFromTail(0),
							prev_dt));
				if (stable_dt >= 2)
					angular_d2v[i].Put(
						derivative2(
							angular_v[i].getFromTail(2),
							angular_v[i].getFromTail(1),
							angular_v[i].getFromTail(0),
							prev_dt));
			}
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

        public static double derivative1_short(double y0, double y1, double dt)    // first derivative
        {
            return (y1 - y0) / dt;
        }

        public static double derivative1_middle(double y0, double y2, double dt)    // first derivative
        {
            return (y2 - y0) / dt * 0.5;
        }

		public static double derivative1(double y0, double y1, double y2, double dt)    // first derivative
		{
			return (y0 - 4 * y1 + 3 * y2) / dt * 0.5;
		}

		public static double derivative2(double y0, double y1, double y2, double dt)    // second derivative
		{
			return (y0 - 2 * y1 + y2) / dt / dt;
		}

		//
		// Short term Model section
		//

		public double[] k_control = new double[3];
        public bool[] stable_channel = new bool[3];     // true if control channel is statically stable in dv_angular

        class ModelException : Exception { }

		public void update_model()
		{
			if (stable_dt < 3)
				return;

			for (int i = 0; i < 3; i++)
			{
                // control diffirential
                double d_control = input_buf[i].getFromTail(1) - input_buf[i].getFromTail(2);
                if (d_control == 0.0)
                    return;
                // get second angular v derivative in previous time slice
                double simple_d2v = derivative1_short(angular_dv[i].getFromTail(2), angular_dv[i].getFromTail(1), prev_dt);
                // channel is statically stable if it's going to zero
                stable_channel[i] = (angular_dv[i].getFromTail(2) * simple_d2v) < 0.0;
                // extrapolate previous angular_dv values
                double extrapolate_dv = 0.0;
                if (stable_channel[i])
                    extrapolate_dv = angular_dv[i].getFromTail(1) +
                        prev_dt * derivative1_middle(angular_dv[i].getFromTail(3), angular_dv[i].getFromTail(1), prev_dt);
                else
                    extrapolate_dv = angular_dv[i].getFromTail(1) +
                        prev_dt * derivative1(angular_dv[i].getFromTail(3), angular_dv[i].getFromTail(2),
                            angular_dv[i].getFromTail(1), prev_dt);
                // get control authority
                double control_authority = (angular_dv[i].getLast() - extrapolate_dv) / d_control;
                k_control[i] = control_authority;
			}
		}

        public double get_short_delta_input_for_axis(int axis, double desired_angular_dv)
        {
            double extrapolate_dv = 0.0;
            if (stable_channel[axis])
                extrapolate_dv = angular_dv[axis].getLast() +
                    prev_dt * derivative1_middle(angular_dv[axis].getFromTail(3), angular_dv[axis].getFromTail(1), prev_dt);
            else
                extrapolate_dv = angular_dv[axis].getLast() +
                    prev_dt * derivative1(angular_dv[axis].getFromTail(3), angular_dv[axis].getFromTail(2),
                        angular_dv[axis].getFromTail(1), prev_dt);
            double d_input = (desired_angular_dv - extrapolate_dv) / k_control[axis];
            return d_input;
        }

        public static double closest(double target, double x1, double x2)
        {
            if (Math.Abs(x1 - target) >= Math.Abs(x2 - target))
                return x2;
            return x1;
        }


        #region GUI

        string module_name = "Instant control model";
        int wnd_id = 34278832;
        protected bool gui_shown = false;
        protected Rect window = new Rect(50.0f, 80.0f, 300.0f, 150.0f);

        public bool IsDrawn()
        {
            return gui_shown;
        }

        public void OnGUI()
        {
            if (!gui_shown)
                return;
            window = GUILayout.Window(wnd_id, window, _drawGUI, module_name);
        }

        public bool ToggleGUI()
        {
            return gui_shown = !gui_shown;
        }

		static readonly string[] axis_names = { "pitch", "roll", "yaw" };

		void _drawGUI(int id)
		{
			GUILayout.BeginVertical();
			for (int i = 0; i < 3; i++)
			{
				GUILayout.Label(axis_names[i] + " ang vel = " + angular_v[i].getLast().ToString("G8"), GUIStyles.labelStyle);
                GUILayout.Label(axis_names[i] + " ang vel d1 = " + angular_dv[i].getLast().ToString("G8"), GUIStyles.labelStyle);
                GUILayout.Label(axis_names[i] + " ang vel d2 = " + angular_d2v[i].getLast().ToString("G8"), GUIStyles.labelStyle);
                GUILayout.Label(axis_names[i] + " K1 = " + k_control[i].ToString("G8"), GUIStyles.labelStyle);
                GUILayout.Label(axis_names[i] + " stable = " + stable_channel[i].ToString(), GUIStyles.labelStyle);
				GUILayout.Space(10);
			}
			GUILayout.EndVertical();
			GUI.DragWindow();
        }

        #endregion
    }
}
