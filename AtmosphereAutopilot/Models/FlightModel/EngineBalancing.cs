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
using System.Reflection;

namespace AtmosphereAutopilot
{

    using Vector = VectorArray.Vector;

    public sealed partial class FlightModel : AutopilotModule
    {
        [AutoGuiAttr("balance_engines", true)]
        [VesselSerializable("balance_engines")]
        public bool balance_engines = false;

        bool[] balancing_possible = new bool[3];

        double[][] torque_coeffs = new double[3][];
        double[] limiters;
        double[] new_limiters;
        double[] limiters_torque_grads;
        double[] limiters_thrust_grads;

        void init_engine_balancing()
        {
            if (!balance_engines)
                return;

            // update engine potentials
            Common.Realloc<double>(ref torque_coeffs[PITCH], engines.Count);
            Common.Realloc<double>(ref torque_coeffs[ROLL], engines.Count);
            Common.Realloc<double>(ref torque_coeffs[YAW], engines.Count);

            Common.Realloc<double>(ref limiters, engines.Count);
            Common.Realloc<double>(ref new_limiters, engines.Count);
            Common.Realloc<double>(ref limiters_torque_grads, engines.Count);
            Common.Realloc<double>(ref limiters_thrust_grads, engines.Count);

            // for pitch only for start
            balancing_possible[PITCH] = false;
            balancing_possible[ROLL] = false;
            balancing_possible[YAW] = false;
            Vector3 mss = Vector3.zero;
            for (int i = 0; i < engines.Count; i++)
            {
                EngineMoment em = engines[i];
                ModuleEngines eng = em.engine;

                // check if balanceble
                if (i == 0)
                {
                    mss = em.potential_torque;
                }
                else
                {
                    Vector3 mss_next = Vector3.zero;
                    mss_next = em.potential_torque;
                    for (int axis = 0; axis < 3; axis++)
                    {
                        if (!balancing_possible[axis])
                        {
                            if (mss_next[axis] * mss[axis] < 0.0f)
                            {
                                balancing_possible[axis] = true;
                                any_gimbals = true;
                            }
                        }
                    }
                    mss = mss_next;
                }

                // initialize if needed
                if (limiters[i] == 0.0)
                    limiters[i] = eng.thrustPercentage * 0.01f;
            }
        }

        [AutoGuiAttr("balance_thrust_w", true, "G5")]
        protected double balance_thrust_w = 1e-3;

        [AutoGuiAttr("balance_descend_k", true, "G5")]
        protected double balance_descend_k = 1e-2;

        [AutoGuiAttr("balance_descend_k_mult", false, "G5")]
        protected double balance_descend_k_mult = 1.0;

        [AutoGuiAttr("balance_err", false, "G5")]
        protected double balance_err = 0.0;

        int unbalancing_counter = 0;

        const double BALANCE_MIN_LIMITER = 1e-4;
        const int BALANCE_MAX_ITER = 10;
        const double BALANCE_ERROR_MARGIN = 1e-4;

        double[] max_potentials = new double[3];

        double[] torque_errors = new double[3];
        double[] new_torque_errors = new double[3];

        void update_engine_balancing()
        {
            if (!balance_engines || limiters == null || limiters.Length < 2)
                return;

            // update params
            for (int axis = 0; axis < 3; axis++)
            {
                max_potentials[axis] = 0.0;
                for (int i = 0; i < engines.Count; i++)
                {
                    // fill coeffs
                    double axis_potential = engines[i].potential_torque[axis] / MOI[axis];
                    torque_coeffs[axis][i] = axis_potential;
                    if (Math.Abs(axis_potential) > max_potentials[axis])
                        max_potentials[axis] = Math.Abs(axis_potential);
                }
            }

            balance_err = 0.0;
            for (int axis = 0; axis < 3; axis++)
            {
                if (balancing_possible[axis])
                {
                    torque_errors[axis] = get_torque_error(limiters, axis);
                    balance_err += torque_errors[axis] * torque_errors[axis];
                }
            }

            if (balance_err <= BALANCE_ERROR_MARGIN)
                return;

            int descend_iter = 0;
            
            do
            {

                // now gradient
                double dot = 0.0;
                for (int i = 0; i < engines.Count; i++)
                {
                    limiters_torque_grads[i] = 0.0;
                    limiters_thrust_grads[i] = 0.0;
                    for (int axis = 0; axis < 3; axis++)
                    {
                        if (balancing_possible[axis])
                        {
                            // torque gradient
                            limiters_torque_grads[i] += torque_errors[axis] * torque_coeffs[axis][i];
                        }
                    }

                    // handle saturation to form bounded optimization
                    if (limiters[i] == 1.0)
                    {
                        if (limiters_torque_grads[i] < 0.0)
                            limiters_torque_grads[i] = 0.0;
                    }

                    if (limiters[i] == BALANCE_MIN_LIMITER)
                    {
                        if (limiters_torque_grads[i] > 0.0)
                            limiters_torque_grads[i] = 0.0;
                    }

                    // gradient for thrust maximization
                    limiters_thrust_grads[i] = -1.0 * Math.Min(1.0, balance_thrust_w / balance_err);

                    // find dot product of those two gradients
                    dot += limiters_torque_grads[i] * limiters_thrust_grads[i];
                }

                //if (dot > 0.0)
                //{
                    // we're all right, we can add thrust gradient to torque one
                    for (int i = 0; i < engines.Count; i++)
                    {
                        limiters_torque_grads[i] += limiters_thrust_grads[i];
                    }
                //}

                // descend while constrained
                for (int i = 0; i < engines.Count; i++)
                {
                    new_limiters[i] = limiters[i] - limiters_torque_grads[i] * balance_descend_k * balance_descend_k_mult;
                    new_limiters[i] = Common.Clamp(new_limiters[i], BALANCE_MIN_LIMITER, 1.0);
                }

                double new_torque_error = 0.0;
                for (int axis = 0; axis < 3; axis++)
                {
                    if (balancing_possible[axis])
                    {
                        new_torque_errors[axis] = get_torque_error(new_limiters, axis);
                        new_torque_error += new_torque_errors[axis] * new_torque_errors[axis];
                    }
                }

                if (new_torque_error < balance_err || unbalancing_counter > 50)
                {
                    if (unbalancing_counter > 50)
                        balance_descend_k_mult = 1.0;
                    else
                        balance_descend_k_mult = Math.Min(1e3, balance_descend_k_mult * 2.0);
                    unbalancing_counter = 0;
                    // descend while constrained
                    for (int i = 0; i < engines.Count; i++)
                    {
                        limiters[i] = new_limiters[i];
                        // push limiter to engine
                        engines[i].engine.thrustPercentage = (float)(limiters[i] * 100.0);
                    }

                    // push changes to other parts of FlightModel
                    for (int axis = 0; axis < 3; axis++)
                    {
                        //engines_torque_k0[axis] = (float)new_torque_errors[axis];
                    }

                    // update error value
                    balance_err = new_torque_error;
                }
                else
                {
                    balance_descend_k_mult = Math.Max(1e-20, balance_descend_k_mult / 10.0);
                    unbalancing_counter++;
                }
                descend_iter++;

            } while (descend_iter < BALANCE_MAX_ITER && balance_err > BALANCE_ERROR_MARGIN);
        }

        double get_torque_error(double[] limiters, int axis)
        {
            double res = 0.0;
            for (int i = 0; i < engines.Count; i++)
            {
                res += torque_coeffs[axis][i] * limiters[i];
            }
            return res;
        }

        [AutoGuiAttr("balancer_steering_k", true, "G4")]
        [VesselSerializable("balancer_steering_k")]
        public double balancer_steering_k = 1.0;

        void postupdate_engine_balancing(FlightCtrlState state)
        {
            if (!balance_engines || limiters == null || limiters.Length < 2)
                return;

            for (int i = 0; i < engines.Count; i++)
            {
                double steering_k = 0.0;
                for (int axis = 0; axis < 3; axis++)
                {
                    if (balancing_possible[axis] && max_potentials[axis] > 0.01)
                        steering_k += torque_coeffs[axis][i] * gimbal_buf[axis].getLast() / max_potentials[axis];
                }
                double new_limit = limiters[i] + balancer_steering_k * steering_k;
                new_limit = Common.Clamp(new_limit, BALANCE_MIN_LIMITER, 1.0);
                engines[i].engine.thrustPercentage = (float)(new_limit * 100.0);
            }
        }

    }

}
