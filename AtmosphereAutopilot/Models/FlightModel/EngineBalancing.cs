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

    public sealed partial class FlightModel : AutopilotModule
    {
        [AutoGuiAttr("balance_engines", true)]
        [VesselSerializable("balance_engines")]
        public bool balance_engines = false;

        bool[] balancing_possible = new bool[3];

        GradientLP optimizer;
        bool balance_first_cycle = false;

        double[][] torque_coeffs = new double[3][];
        double[] limiters;

        int bal_count = 0;

        void init_engine_balancing()
        {
            if (!balance_engines)
                return;

            // update engine potentials
            Common.Realloc<double>(ref torque_coeffs[PITCH], engines.Count);
            Common.Realloc<double>(ref torque_coeffs[ROLL], engines.Count);
            Common.Realloc<double>(ref torque_coeffs[YAW], engines.Count);

            Common.Realloc<double>(ref limiters, engines.Count);

            balancing_possible[PITCH] = false;
            balancing_possible[ROLL] = false;
            balancing_possible[YAW] = false;
            Vector3 mss = Vector3.zero;
            for (int i = 0; i < engines.Count; i++)
            {
                EngineMoment em = engines[i];
                //ModuleEngines eng = em.engine;

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
                            if (mss_next[axis] * mss[axis] < -1.0f)
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
                    limiters[i] = 1.0;
            }

            // initialize optimizer
            bal_count = 0;
            for (int i = 0; i < 3; i++)
                if (balancing_possible[i])
                    bal_count++;

            if (bal_count == 0)
                return;

            if (optimizer == null)
                optimizer = new GradientLP(engines.Count, bal_count);
            else
                optimizer.init(engines.Count, bal_count);

            balance_first_cycle = true;
            for (int i = 0; i < engines.Count; i++)
            {
                // fill x values
                optimizer.x[i, 0] = limiters[i];
            }
        }

        double[] max_potentials = new double[3];
        double max_thrust = 0.0;

        void update_engine_balancing()
        {
            if (!balance_engines || limiters == null || limiters.Length < 2 || bal_count == 0)
                return;

            max_thrust = 1e-6;
            for (int i = 0; i < engines.Count; i++)
            {
                if (engines[i].estimated_max_thrust_unlimited > max_thrust)
                    max_thrust = engines[i].estimated_max_thrust_unlimited;
            }

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

            // fill the optimizer
            int shift = 0;
            for (int axis = 0; axis < 3; axis++)
            {
                if (balancing_possible[axis])
                {
                    // A matrix
                    for (int i = 0; i < engines.Count; i++)
                        optimizer.A[shift, i] = torque_coeffs[axis][i];
                    // b matrix
                    optimizer.b[shift, 0] = 0.0;
                    shift++;
                }
            }

            // thrust maximization goal
            for (int i = 0; i < engines.Count; i++)
                optimizer.c[i, 0] = engines[i].estimated_max_thrust_unlimited / max_thrust;

            // do optimize
            double start, end;
            if (balance_first_cycle)
            {
                optimizer.solve(0.01, out start, out end, 10);
                balance_first_cycle = false;
                optimizer.speed = 1e-3;
            }
            else
            {
                optimizer.solve(100, out start, out end, 10);
                if (end <= 1e-4)
                {
                    for (int i = 0; i < engines.Count; i++)
                        limiters[i] = optimizer.x[i, 0];
                }
            }

            //Debug.Log("A = \r\n" + optimizer.A.ToString());
            //Debug.Log("b = \r\n" + optimizer.b.ToString());
            //Debug.Log("c = \r\n" + optimizer.c.ToString());
            //Debug.Log("x = \r\n" + optimizer.x.ToString());
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
                        steering_k += torque_coeffs[axis][i] * gimbal_buf[axis].getLast() * Math.Abs(engines[i].torque_components[axis]) / max_potentials[axis];
                }
                double new_limit = limiters[i] + balancer_steering_k * steering_k;
                new_limit = Common.Clamp(new_limit, 0.0, 1.0);
                engines[i].engine.thrustPercentage = (float)(new_limit * 100.0);
            }
        }

        [GlobalSerializable("balancing_toggle_key")]
        [AutoHotkeyAttr("Thrust balancing")]
        static KeyCode balancing_toggle_key = KeyCode.None;

        public override void OnUpdate()
        {
            if (Input.GetKeyDown(balancing_toggle_key))
            {
                balance_engines = !balance_engines;
                MessageManager.post_status_message(balance_engines ? "Thrust balancing enabled" : "Thrust balancing disabled");
            }
        }

    }

}
