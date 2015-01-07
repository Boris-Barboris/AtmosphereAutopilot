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
	class InstantControlModel : IAutoGui, IAutoSerializable
	{
		public const int PITCH = 0;
		public const int ROLL = 1;
		public const int YAW = 2;

		Vessel vessel;

		public InstantControlModel(Vessel v)
		{
			vessel = v;
			for (int i = 0; i < 3; i++)
			{
				input_buf[i] = new CircularBuffer<double>(BUFFER_SIZE, true);
				angular_v[i] = new CircularBuffer<double>(BUFFER_SIZE, true);
				angular_dv[i] = new CircularBuffer<double>(BUFFER_SIZE, true);
                k_dv_control[i] = new CircularBuffer<double>(BUFFER_SIZE, true);
			}
			vessel.OnPreAutopilotUpdate += new FlightInputCallback(OnPreAutopilot);
            vessel.OnPostAutopilotUpdate += new FlightInputCallback(OnPostAutopilot);
		}

		static readonly int BUFFER_SIZE = 30;

		public CircularBuffer<double>[] input_buf = new CircularBuffer<double>[3];	// control input value
		public CircularBuffer<double>[] angular_v = new CircularBuffer<double>[3];	// angular v
		public CircularBuffer<double>[] angular_dv = new CircularBuffer<double>[3];	// dv/dt

		double prev_dt = 1.0;		// dt in previous call
		int stable_dt = 0;			// counts amount of stable dt intervals

        void OnPostAutopilot(FlightCtrlState state)		// update control input
		{
			update_control(state);
		}

		void OnPreAutopilot(FlightCtrlState state)	// update all flight characteristics
		{
			double dt = TimeWarp.fixedDeltaTime;
			check_dt(dt);
			update_buffers();
			update_dv_model();
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
			for (int i = 0; i < 3; i++)
			{
				angular_v[i].Put(-vessel.angularVelocity[i]);
				if (stable_dt >= 2)
					angular_dv[i].Put(
                        derivative1(
                            angular_v[i].getFromTail(2),
							angular_v[i].getFromTail(1),
							angular_v[i].getFromTail(0),
							prev_dt));
			}
		}

		void update_control(FlightCtrlState state)
		{
			for (int i = 0; i < 3; i++)
				input_buf[i].Put(getControlFromState(state, i));
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

		public static double derivative2_long(double y0, double y1, double y2, double y3, double dt)
		{
			return (-y0 + 4 * y1 - 5 * y2 + 2 * y3) / dt / dt;
		}

		//
		// Short term model for angular acceleration
		//

        public CircularBuffer<double>[] k_dv_control = new CircularBuffer<double>[3];		// control authority in angular acceleration
        public bool[] dv_stable_channel = new bool[3];      // true if control channel is statically stable in dv_angular
        public bool[] dv_ocsillating = new bool[3];         // true if dv is oscilating
        double[] d_control_acc = new double[3];             // d_control accumulator
        int[] d_control_intervals = new int[3];             // how long it's been accumulated    

		public void update_dv_model()
		{
			if (stable_dt < 3)
				return;

			for (int i = 0; i < 3; i++)
			{
                bool d_control_fixated = false;             // true when input change is finished

                // control diffirential
                double d_control = input_buf[i].getLast() - input_buf[i].getFromTail(1);
                if (d_control == 0.0)
                {
                    // get second angular v derivative in previous time slice
                    double simple_d2v = derivative1_short(angular_dv[i].getFromTail(1), angular_dv[i].getFromTail(0), prev_dt);
                    // channel is dynamically stable if it's angular acceleration going to zero
                    dv_stable_channel[i] = (angular_dv[i].getFromTail(1) * simple_d2v) < 0.0;
                    return;
                }
                else
                {
                    if (d_control * d_control_acc[i] >= 0.0 && Math.Abs(d_control) > min_d_short_control &&
                        d_control_intervals[i] < max_d_control_intervals)
                    {
                        // d_control should be accumulated
                        d_control_acc[i] += d_control;
                        d_control_intervals[i]++;
                    }
                    else
                    {
                        d_control_fixated = true;
                    }
                }

                if (d_control_intervals[i] == 1)    // we'll soon need to ckeck dv for oscillation
                {
                    // Function is oscillating if it's second derivative is changing sign on each step
				    double prev_d3v =
					    derivative2(angular_dv[i].getFromTail(4),
						    angular_dv[i].getFromTail(3),
						    angular_dv[i].getFromTail(2), prev_dt);
				    double cur_d3v =
					    derivative2(angular_dv[i].getFromTail(3),
						    angular_dv[i].getFromTail(2),
						    angular_dv[i].getFromTail(1), prev_dt);
				    if (prev_d3v * cur_d3v < 0.0)
					    dv_ocsillating[i] = true;
				    else
					    dv_ocsillating[i] = false;
                }

                if (d_control_fixated)        // if control change is fixated
                {
                    if (Math.Abs(d_control_acc[i]) > min_d_long_control)        // if it's substantial
                    {
                        // extrapolate previous angular_dv values
                        double extrapolate_dv = 0.0;
                        double control_authority = 0.0;
                        if (!dv_ocsillating[i])
                        {
                            // if acceleratoin is not oscilating, we can use standard derivative approximation
                            int shift = d_control_intervals[i];
                            extrapolate_dv = angular_dv[i].getFromTail(shift) +
                                prev_dt * shift * 
                                derivative1(angular_dv[i].getFromTail(2 + shift),
                                    angular_dv[i].getFromTail(1 + shift), angular_dv[i].getFromTail(0 + shift), prev_dt);
                            // get control authority
                            control_authority = (angular_dv[i].getLast() - extrapolate_dv) / d_control_acc[i];
                        }
                        else
                        {
                            // if it is, wee need to take middles of intervals
                            int shift = d_control_intervals[i];
                            int interval = 0;
                            double dv2 = 0.5 * 
                                (angular_dv[i].getFromTail(shift + interval * 2) + angular_dv[i].getFromTail(shift + interval * 2 + 1));
                            interval++;
                            double dv1 = 0.5 *
                                (angular_dv[i].getFromTail(shift + interval * 2) + angular_dv[i].getFromTail(shift + interval * 2 + 1));
                            interval++;
                            double dv0 = 0.5 *
                                (angular_dv[i].getFromTail(shift + interval * 2) + angular_dv[i].getFromTail(shift + interval * 2 + 1));
                            double d2v_smoothed = derivative1(dv0, dv1, dv2, prev_dt * 2);
                            extrapolate_dv = dv2 + d2v_smoothed * prev_dt * shift;
                            // get control authority
                            control_authority = 
                                (0.5 * (angular_dv[i].getLast() + angular_dv[i].getFromTail(1)) - extrapolate_dv) / d_control_acc[i];
                        }                        
                        if (control_authority > min_authority)
                            k_dv_control[i].Put(control_authority);
                    }

                    d_control_acc[i] = 0.0;         // zero out control change
                    d_control_intervals[i] = 0;
                }
			}
		}

        [AutoGuiAttr("max_d_control_intervals", true, "G6")]
        [GlobalSerializable("max_d_control_intervals")]
        public int max_d_control_intervals = 3;

        [AutoGuiAttr("min_d_long_control", true, "G6")]
        [GlobalSerializable("min_d_long_control")]
        public double min_d_long_control = 0.01;

        [AutoGuiAttr("min_d_short_control", true, "G6")]
        [GlobalSerializable("min_d_short_control")]
        public double min_d_short_control = 0.005;

        [AutoGuiAttr("min_authority", true, "G6")]
        [GlobalSerializable("min_authority")]
        public double min_authority = 0.1;

        public double getDvAuthority(int axis)
        {
            if (k_dv_control[axis].Size > 0)
                return k_dv_control[axis].Average();
            else
                return -1.0;
        }

        public static double closest(double target, double x1, double x2)
        {
            if (Math.Abs(x1 - target) >= Math.Abs(x2 - target))
                return x2;
            return x1;
        }


        #region Serialization

        [GlobalSerializable("window_x")]
        public float WindowLeft { get { return window.xMin; } set { window.xMin = value; } }

        [GlobalSerializable("window_y")]
        public float WindowTop { get { return window.yMin; } set { window.yMin = value; } }

        [GlobalSerializable("window_width")]
        public float WindowWidth { get { return window.width; } set { window.width = value; } }

        public bool Deserialize()
        {
            return AutoSerialization.Deserialize(this, "InstantControlModel",
                KSPUtil.ApplicationRootPath + "GameData/AtmosphereAutopilot/Global_settings.cfg",
                typeof(GlobalSerializable));
        }

        public void Serialize()
        {
            AutoSerialization.Serialize(this, "InstantControlModel",
                KSPUtil.ApplicationRootPath + "GameData/AtmosphereAutopilot/Global_settings.cfg",
                typeof(GlobalSerializable));
        }

        #endregion


        #region GUI

        string module_name = "Instant control model";
        int wnd_id = 34278832;
        protected bool gui_shown = false;
        bool gui_hidden = false;
        protected Rect window = new Rect(50.0f, 80.0f, 240.0f, 150.0f);

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

        public void HideGUI()
        {
            gui_hidden = true;
        }

        public void ShowGUI()
        {
            gui_hidden = false;
        }

		static readonly string[] axis_names = { "pitch", "roll", "yaw" };

		void _drawGUI(int id)
		{
			GUILayout.BeginVertical();
			for (int i = 0; i < 3; i++)
			{
				GUILayout.Label(axis_names[i] + " ang vel = " + angular_v[i].getLast().ToString("G8"), GUIStyles.labelStyleLeft);
                GUILayout.Label(axis_names[i] + " ang vel d1 = " + angular_dv[i].getLast().ToString("G8"), GUIStyles.labelStyleLeft);
                GUILayout.Label(axis_names[i] + " K1 = " + k_dv_control[i].getLast().ToString("G8"), GUIStyles.labelStyleLeft);
                GUILayout.Label(axis_names[i] + " stable = " + dv_stable_channel[i].ToString(), GUIStyles.labelStyleLeft);
                GUILayout.Label(axis_names[i] + " oscillating = " + dv_ocsillating[i].ToString(), GUIStyles.labelStyleLeft);
				GUILayout.Space(5);
			}
            AutoGUI.AutoDrawObject(this);
			GUILayout.EndVertical();
			GUI.DragWindow();
        }

        #endregion
    }
}
