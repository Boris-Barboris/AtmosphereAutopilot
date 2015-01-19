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
                angular_dv_central[i] = new CircularBuffer<double>(BUFFER_SIZE, true);
                k_dv_control[i] = new CircularBuffer<double>(BUFFER_SIZE, true);
			}
			vessel.OnPreAutopilotUpdate += new FlightInputCallback(OnPreAutopilot);
            vessel.OnPostAutopilotUpdate += new FlightInputCallback(OnPostAutopilot);
		}

		static readonly int BUFFER_SIZE = 15;

		public CircularBuffer<double>[] input_buf = new CircularBuffer<double>[3];	// control input value
		public CircularBuffer<double>[] angular_v = new CircularBuffer<double>[3];	// angular v
		public CircularBuffer<double>[] angular_dv = new CircularBuffer<double>[3];	// dv/dt
        public CircularBuffer<double>[] angular_dv_central = new CircularBuffer<double>[3];	// dv/dt

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
                if (stable_dt >= 7)
                {
                    //angular_dv[i].Put(smooth_derivative_hybrid(prev_dt, i));
                    angular_dv[i].Put(
                        derivative1_short(
                            angular_v[i].getFromTail(1),
                            angular_v[i].getFromTail(0),
                            prev_dt));
                    angular_dv_central[i].Put(smooth_derivative_central(prev_dt, i));
                }
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

        public static double extrapolate(double v, double dv, double d2v, double dt)
        {
            return v + dv * dt + 0.5 * d2v * dt * dt;
        }

        public double extrapolate_v(int axis, int order)
        {
            if (stable_dt > order)
                return extrapolate_sequence(angular_v[axis], order);
            else
                return double.NaN;
        }

        public double extrapolate_dv(int axis, int order)
        {
            if (stable_dt > order)
                return extrapolate_sequence(angular_dv[axis], order);
            else
                return double.NaN;
        }

        public double extrapolate_sequence(CircularBuffer<double> buf, int order)
        {
            int points_needed = order + 1;
            if (points_needed < 1)
                throw new ArgumentException("order is wrong");
            if (points_needed == 1)
                return buf.getLast();
            double[] derivatives = new double[order];
            derivative_cascade(derivatives, buf, 1, order);
            double result = buf.getLast();
            for (int i = 0; i < order; i++)
                result += derivatives[i] * Math.Pow(prev_dt, i + 1) / Factorial(i + 1);
            return result;
        }

        static int Factorial(int x)
        {
            return (x == 0) ? 1 : x * Factorial(x - 1);
        }

        void derivative_cascade(double[] ders, CircularBuffer<double> buf, int cur_order, int order)
        {
            if (cur_order != 1)
                throw new InvalidOperationException("cur_order must be 1");
            if (cur_order == order)
            {
                ders[0] = derivative1_short(buf.getFromTail(1), buf.getLast(), prev_dt);
                return;
            }
            double[] order1ders = new double[order];
            for (int i = 0; i < order; i++)
                order1ders[order - i - 1] = derivative1_short(buf.getFromTail(i + 1), buf.getFromTail(i), prev_dt);
            ders[0] = order1ders[order - 1];
            _derivative_cascade(ders, order1ders, cur_order + 1, order);
        }

        void _derivative_cascade(double[] ders, double[] prev_order, int cur_order, int order)
        {
            if (cur_order == order)
            {
                ders[cur_order - 1] = derivative1_short(prev_order[0], prev_order[1], prev_dt);
                return;
            }
            double[] orderders = new double[order - cur_order + 1];
            for (int i = order - cur_order; i >= 0; i--)
                orderders[i] = derivative1_short(prev_order[i], prev_order[i + 1], prev_dt);
            ders[cur_order - 1] = orderders[order - cur_order];
            _derivative_cascade(ders, orderders, cur_order + 1, order);
        }

        /// <summary>
        /// Smooth hybrid noise-robust differentiator, author - Pavel Holoborodko. 
        /// http://www.holoborodko.com/pavel/wp-content/uploads/OneSidedNoiseRobustDifferentiators.pdf
        /// </summary>
        double smooth_derivative_hybrid(double dt, int axis)
        {
            double result =
                6.0 * angular_v[axis].getFromTail(6) +
                1.0 * angular_v[axis].getFromTail(5) +
                -10.0 * angular_v[axis].getFromTail(4) +
                -6.0 * angular_v[axis].getFromTail(3) +
                -8.0 * angular_v[axis].getFromTail(2) +
                5.0 * angular_v[axis].getFromTail(1) +
                12.0 * angular_v[axis].getFromTail(0);
            result /= 28.0 * dt;
            return result;
        }

        /// <summary>
        /// Smooth central noise-robust differentiator, author - Pavel Holoborodko. 
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
		// Short term model for angular acceleration
		//

        public CircularBuffer<double>[] k_dv_control = new CircularBuffer<double>[3];		// control authority in angular acceleration

		public void update_dv_model()
		{
			if (stable_dt < 6)
				return;

			for (int i = 0; i < 3; i++)
			{
                // We'll be working with central derivative, wich is lagging 3 time intervals behind
                // Also we need space for second derivative calculations, so we're 4 steps behind total
                double d_control = 0.5 * (input_buf[i].getFromTail(4) - input_buf[i].getFromTail(5));

                if (Math.Abs(d_control) > min_d_short_control)        // if d_control is substantial
                {
                    // get control authority in acceleration
                    double prev_d2v = derivative1_short(angular_dv_central[i].getFromTail(2), 
                        angular_dv_central[i].getFromTail(1), prev_dt);
                    double cur_d2v = derivative1_short(angular_dv_central[i].getFromTail(1),
                        angular_dv_central[i].getFromTail(0), prev_dt);
                    double control_authority_dv = (cur_d2v - prev_d2v) / d_control;
                    if (control_authority_dv > min_authority_dv)
                        k_dv_control[i].Put(control_authority_dv);
                }
			}
		}

        [AutoGuiAttr("min_d_short_control", true, "G6")]
        [GlobalSerializable("min_d_short_control")]
        public double min_d_short_control = 0.05;

        [AutoGuiAttr("min_authority_dv", true, "G6")]
        [GlobalSerializable("min_authority_dv")]
        public double min_authority_dv = 0.1;

        public double getDvAuthority(int axis)
        {
            if (k_dv_control[axis].Size > 0)
                return k_dv_control[axis].Average();
            else
                return 1.0;
        }

        public double getDvAuthorityInstant(int axis)
        {
            return k_dv_control[axis].getLast();
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
