﻿/*
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
        public class EngineMoment
        {
            public EngineMoment(ModuleEngines m, ModuleGimbal g, float original_gimbal_spd, bool uses_gimbal_spd)
            {
                engine = m;
                gimbal = g;
                this.original_gimbal_spd = original_gimbal_spd;
                this.uses_gimbal_spd = uses_gimbal_spd;
            }
            public ModuleEngines engine;
            public ModuleGimbal gimbal;
            public Vector3 thrust = Vector3.zero;
            public bool uses_gimbal_spd;
            public float original_gimbal_spd;
        }

        public List<EngineMoment> engines = new List<EngineMoment>();

        void get_engines()
        {
            var eng_list = vessel.FindPartModulesImplementing<ModuleEngines>();
            return_gimbals();
            engines.Clear();
            any_gimbals = false;
            any_gimbal_spds = false;
            foreach (var eng in eng_list)
            {
                if (eng.isOperational || eng.finalThrust != 0.0f)
                {
                    ModuleGimbal gimb = eng.part.FindModuleImplementing<ModuleGimbal>();
                    float r_spd = float.PositiveInfinity;
                    bool gmbspd = false;
                    if (gimb != null)
                    {
                        if (!gimb.gimbalLock)
                            any_gimbals = true;
                        r_spd = gimb.gimbalResponseSpeed;
                        if (gimb.useGimbalResponseSpeed)
                        {
                            gmbspd = true;
                            any_gimbal_spds = true;
                        }
                    }
                    engines.Add(new EngineMoment(eng, gimb, r_spd, gmbspd));
                }
            }
            synchronize_gimbals();
        }

        // give back original gimbal response to engines before clearing engine list
        void return_gimbals()
        {            
            foreach (EngineMoment em in engines)
            {
                if (em.gimbal != null)
                {
                    em.gimbal.useGimbalResponseSpeed = em.uses_gimbal_spd;
                    em.gimbal.gimbalResponseSpeed = em.original_gimbal_spd;
                }
            }
        }

        //[AutoGuiAttr("gimbal_spd_norm", false, "G4")]
        public float gimbal_spd_norm = float.PositiveInfinity;

        //[AutoGuiAttr("any_gimbals", false)]
        public bool any_gimbals = false;

        //[AutoGuiAttr("any_gimbal_spds", false)]
        public bool any_gimbal_spds = false;

        // make all gimbals on vessel synchronous
        void synchronize_gimbals()
        {
            gimbal_spd_norm = float.PositiveInfinity;
            if (any_gimbals && any_gimbal_spds)
            {
                // find minimum normalized gimbaling speed
                for (int i = 0; i < engines.Count; i++)
                {
                    ModuleGimbal mg = engines[i].gimbal;
                    if (mg != null)
                    {
                        float norm_spd = engines[i].original_gimbal_spd;
                        if (norm_spd < gimbal_spd_norm)
                            gimbal_spd_norm = norm_spd;
                    }
                }
                // apply it
                for (int i = 0; i < engines.Count; i++)
                {
                    ModuleGimbal mg = engines[i].gimbal;
                    if (mg != null)
                    {
                        mg.useGimbalResponseSpeed = true;
                        mg.gimbalResponseSpeed = gimbal_spd_norm;
                    }
                }
            }
        }

        [AutoGuiAttr("e_torque", false, "G3")]
        public Vector3 engines_torque_principal;

        [AutoGuiAttr("e_thrust", false, "G3")]
        public Vector3 engines_thrust_principal;

        float abs_thrust;

        void update_engine_moments()
        {
            engines_torque_principal = Vector3.zero;
            engines_thrust_principal = Vector3.zero;
            abs_thrust = 0.0f;
            for (int i = 0; i < engines.Count; i++)
            {
                if (engines[i].engine.part.State == PartStates.DEAD || !engines[i].engine.part.isAttached)
                {
                    moments_cycle_counter = 0;      // need to reform parts lists
                    continue;
                }
                Vector3 tpos = Vector3.zero;
                Vector3 tdir = Vector3.zero;
                Vector3 e_thrust = Vector3.zero;
                int tcount = engines[i].engine.thrustTransforms.Count;
                foreach (var trans in engines[i].engine.thrustTransforms)
                {
                    tpos = trans.position - CoM;
                    tdir = -trans.forward;
                    tpos = world_to_cntrl_part * tpos;
                    tdir = world_to_cntrl_part * tdir;
                    Vector3 torque_moment = -Vector3.Cross(tpos, tdir);       // minus because Unity's left-handed
                    engines_torque_principal += torque_moment * engines[i].engine.finalThrust / (float)tcount;
                    engines_thrust_principal += engines[i].engine.finalThrust * tdir / (float)tcount;
                    e_thrust += engines[i].engine.finalThrust / (float)tcount * (-trans.forward);
                }
                abs_thrust += engines[i].engine.finalThrust;
                engines[i].thrust = e_thrust;
            }
        }

        float prev_abs_thrust = 0.0f;

        Vector3 prev_engines_torque;

        [AutoGuiAttr("e_torq_k0", false, "G3")]
        Vector3 engines_torque_k0;

        [AutoGuiAttr("e_torq_k1", false, "G3")]
        Vector3 engines_torque_k1;

        // Stupid linear authority of gimbals, verry approximate but simple and effective.
        // engines_torque = engines_torque_k0 + user_input * engines_torque_k1
        void get_gimbal_authority()
        {
            if (any_gimbals && (prev_abs_thrust != 0.0f) && (abs_thrust != 0.0f) && (gimbal_buf[0].Size >= 2))
            {
                Vector3 scaled_prev_torque = prev_engines_torque / prev_abs_thrust;
                Vector3 scaled_cur_torque = engines_torque_principal / abs_thrust;
                for (int axis = 0; axis < 3; axis++)
                {
                    float cur_cntrl = gimbal_buf[axis].getLast();
                    float last_cntrl = gimbal_buf[axis].getFromTail(1);
                    if (Math.Abs(cur_cntrl - last_cntrl) > 0.05)            // only significant input signal changes are analyzed
                    {
                        float k1 = (scaled_cur_torque[axis] - scaled_prev_torque[axis]) / (cur_cntrl - last_cntrl);
                        if (k1 < 0.0f)
                            k1 = 0.0f;
                        float k0 = scaled_cur_torque[axis] - cur_cntrl * k1;
                        engines_torque_k0[axis] = k0 * abs_thrust;
                        engines_torque_k1[axis] = k1 * abs_thrust;
                    }
                    else
                    {
                        float k1 = engines_torque_k1[axis] / abs_thrust;
                        float k0 = scaled_cur_torque[axis] - cur_cntrl * k1;
                        engines_torque_k0[axis] = k0 * abs_thrust;
                    }
                }
            }
            else
            {
                engines_torque_k0 = engines_torque_principal;
                engines_torque_k1 = Vector3d.zero;
            }
            prev_abs_thrust = abs_thrust;
            prev_engines_torque = engines_torque_principal;
        }
    }
}