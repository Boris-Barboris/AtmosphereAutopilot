/*
Atmosphere Autopilot, plugin for Kerbal Space Program.
Copyright (C) 2015, Baranin Alexander aka Boris-Barboris.
 
Atmosphere Autopilot is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
Atmosphere Autopilot is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
You should have received a copy of the GNU General Public License
along with Atmosphere Autopilot.  If not, see <http://www.gnu.org/licenses/>. 
*/

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using UnityEngine;
using System.IO;

namespace AtmosphereAutopilot
{

    using Vector = VectorArray.Vector;

    public sealed partial class FlightModel : AutopilotModule
	{
        //public Vector3d model_acc = Vector3d.zero;

        LinApprox pitch_aero_torque_model = new LinApprox(2);
        LinApprox pitch_aero_torque_model_gen = new LinApprox(2);
        LinApprox roll_aero_torque_model = new LinApprox(4);
        LinApprox roll_aero_torque_model_gen = new LinApprox(4);
        LinApprox yaw_aero_torque_model = new LinApprox(2);
        LinApprox yaw_aero_torque_model_gen = new LinApprox(2);
        LinApprox pitch_lift_model = new LinApprox(2);
        LinApprox yaw_lift_model = new LinApprox(2);

        const int IMM_BUF_SIZE = 10;

        OnlineLinTrainer pitch_trainer, roll_trainer, yaw_trainer;
        OnlineLinTrainer pitch_lift_trainer, yaw_lift_trainer;

        void initialize_ann_tainers()
        {
            pitch_trainer = new OnlineLinTrainer(pitch_aero_torque_model, pitch_aero_torque_model_gen, IMM_BUF_SIZE, new double[] { 0.01, 0.05 },
                new int[] { 20, 20 }, pitch_input_method, pitch_output_method);
            pitch_trainer.base_gen_weight = 0.0001f;
            pitch_trainer.max_value_decay = 0.001f;
            pitch_trainer.linear_time_decay = 0.005f;
            pitch_trainer.nonlin_time_decay = 0.005f;
            pitch_trainer.linear_err_criteria = 0.02f;
            pitch_trainer.nonlin_trigger = 100;
            pitch_trainer.nonlin_cutoff_time = 1000;

            roll_trainer = new OnlineLinTrainer(roll_aero_torque_model, roll_aero_torque_model_gen, IMM_BUF_SIZE,
                new double[] { 0.01, 0.05, 0.05, 0.05 }, new int[] { 20, 20, 20, 20 }, roll_input_method, roll_output_method);
            roll_trainer.base_gen_weight = 0.001f;
            roll_trainer.max_value_decay = 0.001f;
            roll_trainer.linear_time_decay = 0.005f;
            roll_trainer.nonlin_time_decay = 0.005f;
            roll_trainer.linear_err_criteria = 0.02f;
            roll_trainer.nonlin_trigger = 100;
            roll_trainer.nonlin_cutoff_time = 1000;

            yaw_trainer = new OnlineLinTrainer(yaw_aero_torque_model, yaw_aero_torque_model_gen, IMM_BUF_SIZE, new double[] { 0.01, 0.05 },
                new int[] { 20, 20 }, yaw_input_method, yaw_output_method);
            yaw_trainer.base_gen_weight = 0.0001f;
            yaw_trainer.max_value_decay = 0.001f;
            yaw_trainer.linear_time_decay = 0.005f;
            yaw_trainer.nonlin_time_decay = 0.005f;
            yaw_trainer.linear_err_criteria = 0.02f;
            yaw_trainer.nonlin_trigger = 100;
            yaw_trainer.nonlin_cutoff_time = 1000;

            pitch_lift_trainer = new OnlineLinTrainer(pitch_lift_model, null, IMM_BUF_SIZE, new double[] { 0.01, 0.05 },
                new int[] { 20, 20 }, pitch_lift_input_method, pitch_lift_output_method);
            pitch_lift_trainer.base_gen_weight = 0.1f;
            pitch_lift_trainer.max_value_decay = 0.0005f;
            pitch_lift_trainer.linear_time_decay = 0.002f;
            pitch_lift_trainer.nonlin_time_decay = 0.002f;
            pitch_lift_trainer.linear_err_criteria = 0.02f;
            pitch_lift_trainer.nonlin_trigger = 100;
            pitch_lift_trainer.nonlin_cutoff_time = 1000;

            yaw_lift_trainer = new OnlineLinTrainer(yaw_lift_model, null, IMM_BUF_SIZE, new double[] { 0.01, 0.05 },
                new int[] { 20, 20 }, yaw_lift_input_method, yaw_lift_output_method);
            yaw_lift_trainer.base_gen_weight = 0.1f;
            yaw_lift_trainer.max_value_decay = 0.0005f;
            yaw_lift_trainer.linear_time_decay = 0.002f;
            yaw_lift_trainer.nonlin_time_decay = 0.002f;
            yaw_lift_trainer.linear_err_criteria = 0.02f;
            yaw_lift_trainer.nonlin_trigger = 100;
            yaw_lift_trainer.nonlin_cutoff_time = 1000;
        }

        /// <summary>
        /// Current dynamic pressure = density * air_speed^2
        /// </summary>
        public double dyn_pressure = 1.0;

        // Trainer input methods
        void pitch_input_method(Vector v)
        {
            v[0] = aoa_buf[PITCH].getFromTail(1);   // we need AoA from previous physics frame
            v[1] = csurf_buf[PITCH].getLast();      // same for control surface position (note index diffirence)
        }

        double pitch_output_method()
        {
            return (angular_acc_buf[PITCH].getLast() -
                (reaction_torque[PITCH] * input_buf[PITCH].getLast() + engines_torque[PITCH] +
                get_rcs_torque(PITCH, input_buf[PITCH].getLast())) / MOI[PITCH]) * MOI[PITCH] / dyn_pressure * 1e2;
        }

        void roll_input_method(Vector v)
        {
            v[0] = aoa_buf[YAW].getFromTail(1);
            v[1] = angular_v_buf[ROLL].getFromTail(1) / vessel.srfSpeed * 1e2;
            v[2] = csurf_buf[ROLL].getLast();
            v[3] = csurf_buf[YAW].getLast();
        }

        double roll_output_method()
        {
            return (angular_acc_buf[ROLL].getLast() -
                (reaction_torque[ROLL] * input_buf[ROLL].getLast() + engines_torque[ROLL] +
                get_rcs_torque(ROLL, input_buf[ROLL].getLast())) / MOI[ROLL]) * MOI[ROLL] / dyn_pressure * 1e2;
        }

        void yaw_input_method(Vector v)
        {
            v[0] = aoa_buf[YAW].getFromTail(1);
            v[1] = csurf_buf[YAW].getLast();
        }

        double yaw_output_method()
        {
            return (angular_acc_buf[YAW].getLast() -
                (reaction_torque[YAW] * input_buf[YAW].getLast() + engines_torque[YAW] +
                get_rcs_torque(YAW, input_buf[YAW].getLast())) / MOI[YAW]) * MOI[YAW] / dyn_pressure * 1e2;
        }

        void pitch_lift_input_method(Vector v)
        {
            v[0] = aoa_buf[PITCH].getFromTail(1);
            v[1] = csurf_buf[PITCH].getLast();
        }

        double pitch_lift_output_method()
        {
            return lift_acc / dyn_pressure * 1e3 * sum_mass;        // we approximate lift force, not acceleration
        }

        void yaw_lift_input_method(Vector v)
        {
            v[0] = aoa_buf[YAW].getFromTail(1);
            v[1] = csurf_buf[YAW].getLast();
        }

        double yaw_lift_output_method()
        {
            return slide_acc / dyn_pressure * 1e3 * sum_mass;
        }

        // Training inputs updating
        void update_training_inputs()
        {
            int dt = (int)Math.Round(Time.fixedDeltaTime * 100.0f);
            if (!vessel.LandedOrSplashed && dyn_pressure >= 60.0)
            {
                pitch_trainer.UpdateState(dt);
                pitch_trainer.min_output_value = (float)(0.5 * MOI[PITCH] / dyn_pressure * 1e2);

                roll_trainer.UpdateState(dt);
                roll_trainer.min_output_value = (float)(1.0 * MOI[ROLL] / dyn_pressure * 1e2);

                yaw_trainer.UpdateState(dt);
                yaw_trainer.min_output_value = (float)(0.5 * MOI[YAW] / dyn_pressure * 1e2);

                pitch_lift_trainer.UpdateState(dt);
                pitch_lift_trainer.min_output_value = (float)(10.0 / dyn_pressure * 1e3 * sum_mass);

                yaw_lift_trainer.UpdateState(dt);
                yaw_lift_trainer.min_output_value = (float)(2.0 / dyn_pressure * 1e3 * sum_mass);
            }
        }

        // Training methods
        //[AutoGuiAttr("pitch_cpu", false)]
        int pitch_cpu = 0;

        //[AutoGuiAttr("roll_cpu", false)]
        int roll_cpu = 5;

        //[AutoGuiAttr("yaw_cpu", false)]
        int yaw_cpu = 0;

        //[AutoGuiAttr("pitch_lift_cpu", false)]
        int pitch_lift_cpu = 0;

        //[AutoGuiAttr("yaw_lift_cpu", false)]
        int yaw_lift_cpu = 5;

        //[AutoGuiAttr("CPU per update", true)]
        int CPU_TIME_FOR_FIXEDUPDATE = 5;

        const int TORQUE_DESCEND_COST = 10;
        const int LIFT_DESCEND_COST = 10;

        void update_cpu()
        {
            Interlocked.Add(ref pitch_cpu, CPU_TIME_FOR_FIXEDUPDATE);
            Interlocked.Add(ref roll_cpu, CPU_TIME_FOR_FIXEDUPDATE);
            Interlocked.Add(ref yaw_cpu, CPU_TIME_FOR_FIXEDUPDATE);
            Interlocked.Add(ref pitch_lift_cpu, CPU_TIME_FOR_FIXEDUPDATE);
            Interlocked.Add(ref yaw_lift_cpu, CPU_TIME_FOR_FIXEDUPDATE);
        }

        bool train_pitch_ann()
        {
            if (pitch_cpu >= TORQUE_DESCEND_COST)
            {
                Interlocked.Add(ref pitch_cpu, -TORQUE_DESCEND_COST);
                pitch_trainer.Train();
                return true;
            }
            return false;
        }

        bool train_roll_ann()
        {
            if (roll_cpu >= TORQUE_DESCEND_COST)
            {
                Interlocked.Add(ref roll_cpu, -TORQUE_DESCEND_COST);
                roll_trainer.Train();
                return true;
            }
            return false;
        }

        bool train_yaw_ann()
        {
            if (yaw_cpu >= TORQUE_DESCEND_COST)
            {
                Interlocked.Add(ref yaw_cpu, -TORQUE_DESCEND_COST);
                yaw_trainer.Train();
                return true;
            }
            return false;
        }

        bool train_pitch_lift()
        {
            if (pitch_lift_cpu >= LIFT_DESCEND_COST)
            {
                Interlocked.Add(ref pitch_lift_cpu, -LIFT_DESCEND_COST);
                pitch_lift_trainer.Train();
                return true;
            }
            return false;
        }

        bool train_yaw_lift()
        {
            if (yaw_lift_cpu >= LIFT_DESCEND_COST)
            {
                Interlocked.Add(ref yaw_lift_cpu, -LIFT_DESCEND_COST);
                yaw_lift_trainer.Train();
                return true;
            }
            return false;
        }

        void update_model_acc()
        {
            pitch_aero_torque_model.update_from_training();
            pitch_aero_torque_model_gen.update_from_training();
            roll_aero_torque_model.update_from_training();
            roll_aero_torque_model_gen.update_from_training();
            yaw_aero_torque_model.update_from_training();
            yaw_aero_torque_model_gen.update_from_training();
            pitch_lift_model.update_from_training();
            yaw_lift_model.update_from_training();
        }
    }
}
