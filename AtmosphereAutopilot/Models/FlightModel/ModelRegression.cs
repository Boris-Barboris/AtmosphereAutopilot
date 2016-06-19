/*
Atmosphere Autopilot, plugin for Kerbal Space Program.
Copyright (C) 2015-2016, Baranin Alexander aka Boris-Barboris.
 
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
    using TrainerTask = OnlineLinTrainer.LinApproxTask;

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

        TrainerTask pitch_aero_torque_task = new TrainerTask();
        TrainerTask pitch_aero_torque_task_gen = new TrainerTask();
        TrainerTask roll_aero_torque_task = new TrainerTask();
        TrainerTask roll_aero_torque_task_gen = new TrainerTask();
        TrainerTask yaw_aero_torque_task = new TrainerTask();
        TrainerTask yaw_aero_torque_task_gen = new TrainerTask();
        TrainerTask pitch_lift_task = new TrainerTask();
        TrainerTask yaw_lift_task = new TrainerTask();

        const int IMM_BUF_SIZE = 10;

        OnlineLinTrainer pitch_trainer, roll_trainer, yaw_trainer;
        OnlineLinTrainer pitch_lift_trainer, yaw_lift_trainer;

        void initialize_lin_tainers()
        {
            // Initial dumb aero values
            if (HasControlSurfaces)
            {
                pitch_aero_torque_model.tpars[2] = 10.0;
                pitch_aero_torque_model_gen.tpars[2] = 10.0;
                roll_aero_torque_model.tpars[3] = 10.0;
                roll_aero_torque_model_gen.tpars[3] = 10.0;
                yaw_aero_torque_model.tpars[2] = 10.0;
                yaw_aero_torque_model_gen.tpars[2] = 10.0;
            }
            pitch_lift_model.tpars[2] = sum_mass;
            yaw_lift_model.tpars[2] = sum_mass;

            //  Initialize pitch rotation trainer
            pitch_aero_torque_task.linmodel = pitch_aero_torque_model;
            pitch_aero_torque_task.base_gen_weight = 0.0001f;
            pitch_aero_torque_task.linear_time_decay = 0.005f;
            pitch_aero_torque_task.nonlin_time_decay = 0.01f;
            pitch_aero_torque_task.validator = (a, b) => pitch_validator(pitch_aero_torque_model, a, b);

            pitch_aero_torque_task_gen.linmodel = pitch_aero_torque_model_gen;
            pitch_aero_torque_task_gen.base_gen_weight = 0.5f;
            pitch_aero_torque_task_gen.linear_time_decay = 0.005f;
            pitch_aero_torque_task_gen.nonlin_time_decay = 0.01f;
            pitch_aero_torque_task_gen.validator = (a, b) => pitch_validator(pitch_aero_torque_model_gen, a, b);

            pitch_trainer = new OnlineLinTrainer(IMM_BUF_SIZE, new double[] { 0.01, 0.05 },
                new int[] { 20, 20 }, pitch_input_method, pitch_output_method, pitch_aero_torque_task_gen, pitch_aero_torque_task);
            pitch_trainer.max_value_decay = 0.001f;
            pitch_trainer.linear_err_criteria = 0.01f;
            pitch_trainer.nonlin_trigger = 100;
            pitch_trainer.nonlin_cutoff_time = 500;

            //  Initialize roll rotation trainer
            roll_aero_torque_task.linmodel = roll_aero_torque_model;
            roll_aero_torque_task.base_gen_weight = 0.001f;
            roll_aero_torque_task.linear_time_decay = 0.005f;
            roll_aero_torque_task.nonlin_time_decay = 0.01f;
            roll_aero_torque_task.validator = (a, b) => roll_validator(roll_aero_torque_model, a, b);

            roll_aero_torque_task_gen.linmodel = roll_aero_torque_model_gen;
            roll_aero_torque_task_gen.base_gen_weight = 0.5f;
            roll_aero_torque_task_gen.linear_time_decay = 0.005f;
            roll_aero_torque_task_gen.nonlin_time_decay = 0.01f;
            roll_aero_torque_task_gen.validator = (a, b) => roll_validator(roll_aero_torque_model_gen, a, b);

            roll_trainer = new OnlineLinTrainer(IMM_BUF_SIZE, new double[] { 0.01, 0.05, 0.05, 0.05 }, 
                new int[] { 15, 15, 15, 15 }, roll_input_method, roll_output_method, roll_aero_torque_task_gen, roll_aero_torque_task);
            roll_trainer.max_value_decay = 0.001f;
            roll_trainer.linear_err_criteria = 0.02f;
            roll_trainer.nonlin_trigger = 100;
            roll_trainer.nonlin_cutoff_time = 500;

            //  Initialize yaw rotation trainer
            yaw_aero_torque_task.linmodel = yaw_aero_torque_model;
            yaw_aero_torque_task.base_gen_weight = 0.0001f;
            yaw_aero_torque_task.linear_time_decay = 0.005f;
            yaw_aero_torque_task.nonlin_time_decay = 0.01f;
            yaw_aero_torque_task.validator = (a, b) => pitch_validator(yaw_aero_torque_model, a, b);

            yaw_aero_torque_task_gen.linmodel = yaw_aero_torque_model_gen;
            yaw_aero_torque_task_gen.base_gen_weight = 0.5f;
            yaw_aero_torque_task_gen.linear_time_decay = 0.005f;
            yaw_aero_torque_task_gen.nonlin_time_decay = 0.01f;
            yaw_aero_torque_task_gen.validator = (a, b) => pitch_validator(yaw_aero_torque_model_gen, a, b);

            yaw_trainer = new OnlineLinTrainer(IMM_BUF_SIZE, new double[] { 0.01, 0.05 },
                new int[] { 20, 20 }, yaw_input_method, yaw_output_method, yaw_aero_torque_task_gen, yaw_aero_torque_task);
            yaw_trainer.max_value_decay = 0.001f;
            yaw_trainer.linear_err_criteria = 0.01f;
            yaw_trainer.nonlin_trigger = 100;
            yaw_trainer.nonlin_cutoff_time = 500;

            //  Initialize pitch lift trainer
            pitch_lift_task.linmodel = pitch_lift_model;
            pitch_lift_task.base_gen_weight = 2.0f;
            pitch_lift_task.linear_time_decay = 0.002f;
            pitch_lift_task.nonlin_time_decay = 0.005f;
            pitch_lift_task.validator = (a, b) => pitch_lift_validator(pitch_lift_model, a, b);

            pitch_lift_trainer = new OnlineLinTrainer(IMM_BUF_SIZE, new double[] { 0.01, 0.05 },
                new int[] { 20, 20 }, pitch_lift_input_method, pitch_lift_output_method, pitch_lift_task);
            pitch_lift_trainer.max_value_decay = 0.0005f;
            pitch_lift_trainer.linear_err_criteria = 0.0005f;
            pitch_lift_trainer.nonlin_trigger = 100;
            pitch_lift_trainer.nonlin_cutoff_time = 500;

            //  Initialize yaw lift trainer
            yaw_lift_task.linmodel = yaw_lift_model;
            yaw_lift_task.base_gen_weight = 2.0f;
            yaw_lift_task.linear_time_decay = 0.002f;
            yaw_lift_task.nonlin_time_decay = 0.005f;
            yaw_lift_task.validator = (a, b) => pitch_lift_validator(yaw_lift_model, a, b);

            yaw_lift_trainer = new OnlineLinTrainer(IMM_BUF_SIZE, new double[] { 0.01, 0.05 },
                new int[] { 20, 20 }, yaw_lift_input_method, yaw_lift_output_method, yaw_lift_task);
            yaw_lift_trainer.max_value_decay = 0.0005f;
            yaw_lift_trainer.linear_err_criteria = 0.001f;
            yaw_lift_trainer.nonlin_trigger = 100;
            yaw_lift_trainer.nonlin_cutoff_time = 500;
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
                (reaction_torque[PITCH] * input_buf[PITCH].getLast() + engines_torque_principal[PITCH] +
                get_rcs_torque(PITCH, input_buf[PITCH].getLast())) / MOI[PITCH]) * MOI[PITCH] / dyn_pressure * 1e2;
        }

        bool pitch_validator(LinApprox linmodel, bool[] changed, bool[] reassigned)
        {
            if (HasControlSurfaces)
                if (linmodel.tpars[2] < 1e-6)       // authority can't be negative or zero
                {
                    linmodel.tpars[2] = MOI[0] / 1000.0;
                    changed.CopyTo(reassigned, 0);
                    reassigned[1] = false;
                    return true;
                }
                else { }
            else
                linmodel.tpars[2] = 0.0;
            return false;
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
                (reaction_torque[ROLL] * input_buf[ROLL].getLast() + engines_torque_principal[ROLL] +
                get_rcs_torque(ROLL, input_buf[ROLL].getLast())) / MOI[ROLL]) * MOI[ROLL] / dyn_pressure * 1e2;
        }

        bool roll_validator(LinApprox linmodel, bool[] changed, bool[] reassigned)
        {
            bool ret = false;
            changed.CopyTo(reassigned, 0);
            if (linmodel.tpars[2] > 0.0)      // friction must kill rotation
            {
                linmodel.tpars[2] = 0.0;
                reassigned[1] = false;
                ret = true;
            }
            if (HasControlSurfaces)
            {
                if (linmodel.tpars[3] < 1e-6)      // authority can't be negative or zero
                {
                    linmodel.tpars[3] = MOI[ROLL] / 1000.0;
                    reassigned[2] = false;
                    ret = true;
                }
            }
            else
                linmodel.tpars[3] = 0.0;
            return ret;
        }

        void yaw_input_method(Vector v)
        {
            v[0] = aoa_buf[YAW].getFromTail(1);
            v[1] = csurf_buf[YAW].getLast();
        }

        double yaw_output_method()
        {
            return (angular_acc_buf[YAW].getLast() -
                (reaction_torque[YAW] * input_buf[YAW].getLast() + engines_torque_principal[YAW] +
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

        bool pitch_lift_validator(LinApprox linmodel, bool[] changed, bool[] reassigned)
        {
            // plane airframe must provide positive lift derivative
            double sign = Math.Sign(Math.Abs(AoA(PITCH)) - 90.0 * dgr2rad);
            if (linmodel.tpars[1] * sign > 0.0)
            {
                changed.CopyTo(reassigned, 0);
                linmodel.tpars[1] = -sign * sum_mass / 10.0;
                reassigned[0] = false;
                return true;
            }
            return false;
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
            int dt = (int)Math.Max(1, Math.Round(Time.fixedDeltaTime * 100.0f));
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
