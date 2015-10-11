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
        struct EngineMoment
        {
            public EngineMoment(ModuleEngines m, ModuleGimbal g, bool turned_off_gimbal_spd)
            {
                engine = m;
                gimbal = g;
                this.tunred_off_gimbal_spd = turned_off_gimbal_spd;
            }
            public ModuleEngines engine;
            public ModuleGimbal gimbal;
            public bool tunred_off_gimbal_spd;
        }

        List<EngineMoment> engines = new List<EngineMoment>();

        void get_engines()
        {
            var eng_list = vessel.FindPartModulesImplementing<ModuleEngines>();
            return_gimbals();
            engines.Clear();
            foreach (var eng in eng_list)
            {
                if (eng.isOperational || eng.finalThrust != 0.0f)
                {
                    ModuleGimbal gimb = eng.part.FindModuleImplementing<ModuleGimbal>();
                    bool toff = false;
                    if (gimb != null)
                        if (gimb.useGimbalResponseSpeed)
                        {
                            gimb.useGimbalResponseSpeed = false;        // turn off gimbal response speed
                            toff = true;
                        }
                    engines.Add(new EngineMoment(eng, gimb, toff));
                }
            }
        }

        void return_gimbals()
        {
            // give back gimbal response to engines before clearing engine list
            foreach (EngineMoment em in engines)
            {
                if (em.gimbal != null && em.tunred_off_gimbal_spd)
                    em.gimbal.useGimbalResponseSpeed = true;
            }
        }

        [AutoGuiAttr("e_torque", false, "G4")]
        public Vector3 engines_torque;

        [AutoGuiAttr("e_thrust", false, "G4")]
        public Vector3 engines_thrust;

        float abs_thrust;

        void update_engine_moments()
        {
            engines_torque = Vector3.zero;
            engines_thrust = Vector3.zero;
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
                int tcount = engines[i].engine.thrustTransforms.Count;
                foreach (var trans in engines[i].engine.thrustTransforms)
                {
                    tpos = trans.position - CoM;
                    tdir = -trans.forward;
                    tpos = world_to_cntrl_part * tpos;
                    tdir = world_to_cntrl_part * tdir;
                    Vector3 torque_moment = -Vector3.Cross(tpos, tdir);       // minus because Unity's left-handed
                    engines_torque += torque_moment * engines[i].engine.finalThrust / (float)tcount;
                    engines_thrust += engines[i].engine.finalThrust * tdir / (float)tcount;
                }
                abs_thrust += engines[i].engine.finalThrust;
            }
        }

        float prev_abs_thrust = 0.0f;

        Vector3 prev_engines_torque;

        [AutoGuiAttr("e_torq_k0", false, "G4")]
        Vector3 engines_torque_k0;

        [AutoGuiAttr("e_torq_k1", false, "G4")]
        Vector3 engines_torque_k1;

        // Stupid linear authority of gimbals
        // engines_torque = engines_torque_k0 + user_input * engines_torque_k1
        void get_gimbal_authority()
        {
            if ((prev_abs_thrust != 0.0f) && (abs_thrust != 0.0f) && (input_buf[0].Size >= 2))
            {
                Vector3 scaled_prev_torque = prev_engines_torque / prev_abs_thrust;
                Vector3 scaled_cur_torque = engines_torque / abs_thrust;
                for (int axis = 0; axis < 3; axis++)
                {
                    float cur_cntrl = input_buf[axis].getLast();
                    float last_cntrl = input_buf[axis].getFromTail(1);
                    if (Math.Abs(cur_cntrl - last_cntrl) > 0.05)
                    {
                        float k1 = (scaled_cur_torque[axis] - scaled_prev_torque[axis]) / (cur_cntrl - last_cntrl);
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
                engines_torque_k0 = engines_torque;
                engines_torque_k1 = Vector3d.zero;
            }
            prev_abs_thrust = abs_thrust;
            prev_engines_torque = engines_torque;
        }
    }
}
