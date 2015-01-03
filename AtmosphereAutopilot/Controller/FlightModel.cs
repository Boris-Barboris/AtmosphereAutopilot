using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;
using System.IO;

namespace AtmosphereAutopilot
{

	/// <summary>
	/// Class for current vessel flight model calculations
	/// </summary>
	class FlightModel
	{
		/// <summary>
		/// Enumeration of all handeled flight parameters
		/// </summary>
		enum FCharacter
		{
			pitch = 0,		// pitch control value
			roll,			// roll control value
			yaw,			// yaw control value
			aoa_pitch,
			aoa_slide,
			com_horiz,		// angle between vessel forward and horizont
			surf_roll,		// angle between vessel wings and horizont
			dyn_pressure
		}

        public const int Param_count = 8;

        /// <summary>
        /// Enumeration of parameters model is trying to predict
        /// </summary>
        enum FControl
        {
            avd_pitch = 0,  // pitch angular velocity derivative
            avd_roll,       // roll angular velocity derivative
            avd_yaw
        }

		public FlightModel(Vessel vessel)
		{
            this.vessel = vessel;
			for (int i = 0; i < Param_count; i++)
				telemetry[i] = new CircularBuffer<double>(Steps_remembered, true);
            for (int i = 0; i < 3; i++)
            {
                angular_velocities[i] = new CircularBuffer<double>(Steps_remembered, true);
                angular_derivatives[i] = new CircularBuffer<double>(Steps_remembered, true);
                model_linear_k[i] = new double[Param_count];
                model_parameters_importance[i] = new double[Param_count];
            }
            set_inportance();
            vessel.OnFlyByWire += new FlightInputCallback(OnFlyByWire);

            // GUI
            //labelstyle.fontSize = 7;
            //labelstyle.margin = new RectOffset(2, 2, 2, 2);
		}

        Vessel vessel;
		
		public const int Steps_remembered = 100;        // time slices to remember. Affects performance
        int dt_stable = 0;
        double last_dt = 1.0;           // last delta_t

		public void OnFlyByWire(FlightCtrlState state)
		{
            if (vessel.checkLanded())           // ground breaks the model
                return;

            // record flight readings
			telemetry[(int)FCharacter.pitch].Put(state.pitch);
			telemetry[(int)FCharacter.roll].Put(state.roll);
			telemetry[(int)FCharacter.yaw].Put(state.yaw);

            Vector3 tmpVec = vessel.ReferenceTransform.up * Vector3.Dot(vessel.ReferenceTransform.up, vessel.srf_velocity.normalized) + vessel.ReferenceTransform.forward * Vector3.Dot(vessel.ReferenceTransform.forward, vessel.srf_velocity.normalized);
            double aoa = Math.Asin(Vector3.Dot(vessel.ReferenceTransform.forward.normalized, tmpVec.normalized));
            telemetry[(int)FCharacter.aoa_pitch].Put(aoa);

            telemetry[(int)FCharacter.aoa_slide].Put(0);

            double pitch = Math.Abs(Vector3.Cross(vessel.transform.up.normalized, vessel.upAxis).magnitude);
            telemetry[(int)FCharacter.com_horiz].Put(pitch);

            telemetry[(int)FCharacter.surf_roll].Put(0);

            telemetry[(int)FCharacter.dyn_pressure].Put(vessel.srf_velocity.sqrMagnitude * FlightGlobals.getStaticPressure());

            angular_velocities[(int)FControl.avd_pitch].Put(-vessel.angularVelocity.x);
            angular_velocities[(int)FControl.avd_roll].Put(-vessel.angularVelocity.y);
            angular_velocities[(int)FControl.avd_yaw].Put(-vessel.angularVelocity.z);

            // update
            update_derivatives();
            predict();
            solve();
		}

        // Flight characteristics buffers
		CircularBuffer<double>[] telemetry = new CircularBuffer<double>[Param_count];
        CircularBuffer<double>[] angular_velocities = new CircularBuffer<double>[3];
        CircularBuffer<double>[] angular_derivatives = new CircularBuffer<double>[3];

        // Model characteristics buffers
        public double[][] model_linear_k = new double[3][];                        // linear gain coeffitients
        double[][] model_parameters_importance = new double[3][];           // importance of flight parameters
        double[] error = new double[3];                                     // errors of angular vel derivatives prediction
        int solve_count = 0;                                                // matrix solve counter. Used for stabilizing model
        int solve_memory = 20;                                              // how many last solutions will be accounted when building a model

        void update_derivatives()
        {
            double dt = TimeWarp.fixedDeltaTime;
            if (Math.Abs(dt / last_dt - 1.0) < 0.1)
            {
                // dt is roughly constant
                dt_stable = Math.Min(dt_stable + 1, 100);       // increase dt stability counter
                if (dt_stable >= 1)                             // if dt is stable long enough
                    for (int i = 0; i < 3; i++)
                        if (angular_velocities[i].Size >= 3)
                            angular_derivatives[i].Put(derivative1(angular_velocities[i].getFromTail(2),
                                angular_velocities[i].getFromTail(1), angular_velocities[i].getFromTail(0), dt));
            }
            else
            {
                // new delta_t
                dt_stable = 0;
                solve_count = 0;
            }
            last_dt = dt;
        }

        void predict()
        {
            double[] prediction = new double[3];
            for (int i = 0; i < 3; i++)
            {
                prediction[i] = 0.0;
                for (int j = 0; j < Param_count; j++)
                    prediction[i] += model_linear_k[i][j] * telemetry[j].getFromTail(0);
            }
            for (int i = 0; i < 3; i++)
                error[i] = prediction[i] - angular_derivatives[i].getFromTail(0);
        }

        string[] importance_str = new string[Param_count];

        void set_inportance()
        {
            importance_str[(int)FCharacter.pitch] = (model_parameters_importance[0][(int)FCharacter.pitch] = 1.0).ToString("G8");
            importance_str[(int)FCharacter.aoa_pitch] = (model_parameters_importance[0][(int)FCharacter.aoa_pitch] = 1.0).ToString("G8");
            importance_str[(int)FCharacter.com_horiz] = (model_parameters_importance[0][(int)FCharacter.com_horiz] = 1.0).ToString("G8");
            importance_str[(int)FCharacter.dyn_pressure] = (model_parameters_importance[0][(int)FCharacter.dyn_pressure] = 1.0).ToString("G8");
        }

        int solve_cycle_counter = 0;
        void solve()
        {
            solve_cycle_counter = (solve_cycle_counter + 1) % Steps_remembered;
            if (dt_stable < Param_count + 1 || solve_cycle_counter != 0)
                return;

            try
            {
                // Linear least squares method
                // Parameter vector = (Xt * X)^-1 * Xt * y

                //
                // Pitch
                //
                bool[] non_zero = new bool[Param_count];            // get non-zero parameters for pitch
                int non_zero_pars = 0;
                for (int j = 0; j < Param_count; j++)
                    for (int i = 0; i < Steps_remembered; i++)
                    {
                        if (telemetry[j][i] * model_parameters_importance[0][j] != 0.0)
                        {
                            non_zero[j] = true;
                            non_zero_pars++;
                            break;
                        }
                    }
                if (non_zero_pars == 0)
                    return;

                Matrix tel_matrix = new Matrix(Steps_remembered, non_zero_pars);
                for (int i = 0; i < Steps_remembered; i++)
                {
                    for (int j = 0, k = 0; j < Param_count; j++)
                        if (non_zero[j])
                        {
                            tel_matrix[i, k] = telemetry[j][i] * model_parameters_importance[0][j];
                            k++;
                        }
                }
                Matrix tel_m_transposed = Matrix.Transpose(tel_matrix);
                Matrix inverted = (tel_m_transposed * tel_matrix).Invert();
                Matrix ang_dev_pitch = new Matrix(Steps_remembered, 1);
                for (int i = 0; i < Steps_remembered; i++)
                    ang_dev_pitch[i, 0] = angular_derivatives[(int)FControl.avd_pitch][i];
                Matrix result_vector = inverted * tel_m_transposed * ang_dev_pitch;

                // Apply results
                for (int i = 0, j = 0; i < Param_count; i++)
                    if (non_zero[i])
                    {
                        double new_koef = result_vector[j, 0];
                        j++;
                        if (!double.IsInfinity(new_koef) && !double.IsNaN(new_koef))
                            model_linear_k[(int)FControl.avd_pitch][i] = 
                                (new_koef + model_linear_k[(int)FControl.avd_pitch][i] * solve_count) / (solve_count + 1);
                    }

                solve_count = Math.Min(solve_count + 1, solve_memory);
            }
            catch (Exception e)
            {
                Debug.Log("[Autopilot]: " + e.Message);
            }

            // rows are tme slices
            // columns are linear coeffitients
            // free column is angular derivative
            //double[,] matrix = new double[2 + 1, 2 + 2];
            //for (int t = 0; t < 2 + 1; t++)
            //{
            //    matrix[t, 2 + 1] = angular_derivatives[0].getFromTail(t);
            //    matrix[t, 2] = 1.0;
            //    matrix[t, 0] = telemetry[(int)FCharacter.pitch].getFromTail(t);
            //    matrix[t, 1] = telemetry[(int)FCharacter.aoa_pitch].getFromTail(t);
            //}
            //if (LinearEquationSolver.Solve(matrix))
            //{
            //    double aoa_k = matrix[0, 3];
            //    double pitch_k = matrix[1, 3];
            //    double unknown_k = matrix[2, 3];

            //    // apply changes in model
            //    if (!double.IsInfinity(aoa_k) && !double.IsNaN(aoa_k))
            //        model_linear_k[(int)FCharacter.aoa_pitch] = (aoa_k + model_linear_k[(int)FCharacter.aoa_pitch] * solve_count) /
            //            (solve_count + 1);
            //    if (!double.IsInfinity(pitch_k) && !double.IsNaN(pitch_k))
            //        model_linear_k[(int)FCharacter.pitch] = (pitch_k + model_linear_k[(int)FCharacter.pitch] * solve_count) /
            //            (solve_count + 1);
            //    if (!double.IsInfinity(unknown_k) && !double.IsNaN(unknown_k))
            //        model_linear_k[Param_count] = (unknown_k + model_linear_k[Param_count] * solve_count) /
            //            (solve_count + 1);
                
            //    solve_count = Math.Min(solve_count + 1, solve_memory);
            //}
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
        // GUI
        //

        //GUIStyle labelstyle = new GUIStyle(GUI.skin.label);

        protected bool gui_shown = false;
        public void toggleGUI()
        {
            gui_shown = !gui_shown;
            if (gui_shown)
                RenderingManager.AddToPostDrawQueue(5, drawGUI);
            else
                RenderingManager.RemoveFromPostDrawQueue(5, drawGUI);
        }

        protected Rect window = new Rect(250.0f, 50.0f, 450.0f, 250.0f);

        void drawGUI()
        {
            if (!gui_shown)
                return;
            window = GUILayout.Window(77347, window, _drawGUI, "FlightModel");
        }

        void _drawGUI(int id)
        {
            GUILayout.BeginVertical();

            for (int i = 0; i < Param_count; i++)
            {
                GUILayout.BeginHorizontal();
                try
                {
                    GUILayout.Label(Enum.GetName(typeof(FCharacter), (FCharacter)i) + " = " +
                        telemetry[i].getFromTail(0).ToString("G8"));
                }
                catch { }
                try
                {
                    for (int j = 0; j < 1; j++)
                        GUILayout.Label("Linear K = " + model_linear_k[0][i].ToString("G8"));
                }
                catch { }
                GUILayout.EndHorizontal();
            }
            for (int i = 0; i < 3; i++)
            {
                try
                {
                    GUILayout.Label(Enum.GetName(typeof(FControl), (FControl)i) + " = " +
                        angular_derivatives[i].getFromTail(0).ToString("G8") + " error = " + error[i].ToString("G8"));
                }
                catch { }
            }
            for (int i = 0; i < Param_count; i++)
            {
                if (importance_str[i] == null)
                    continue;
                GUILayout.BeginHorizontal();
                try
                {
                    GUILayout.Label(Enum.GetName(typeof(FCharacter), (FCharacter)i) + " importance = ");
                    importance_str[i] = GUILayout.TextField(importance_str[i]);
                    model_parameters_importance[0][i] = double.Parse(importance_str[i]);
                }
                catch { }
                GUILayout.EndHorizontal();
            }

            GUILayout.EndVertical();
            GUI.DragWindow();
        }
	}
}
