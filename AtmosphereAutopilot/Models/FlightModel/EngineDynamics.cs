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
        public class EngineMoment
        {
            public EngineMoment(ModuleEngines m, IGimbal g)
            {
                engine = m;
                gimbal = g;
            }
            public ModuleEngines engine;
            public IGimbal gimbal;
            public Vector3 thrust = Vector3.zero;
            public Vector3 potential_torque = Vector3.zero;
            public Vector3 torque_components = Vector3.zero;
            public double estimated_max_thrust = 0.0;
            public double estimated_max_thrust_unlimited = 0.0;
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
                    IGimbal gimb = null;
                    foreach (Type moduleType in AtmosphereAutopilot.gimbal_module_wrapper_map.Keys)
                    {
                        PartModule gimbal_module = findModuleByName(eng.part, moduleType.Name);
                        if (gimbal_module != null)
                        {
                            gimb = (IGimbal)AtmosphereAutopilot.gimbal_module_wrapper_map[moduleType].
                                Invoke(new []{ gimbal_module });
                            if (gimb.Active)
                            {
                                any_gimbals = true;
                                if (gimb.UseGimbalSpeed)
                                    any_gimbal_spds = true;
                            }
                            break;
                        }
                    }
                    engines.Add(new EngineMoment(eng, gimb));
                }
            }
            synchronize_gimbals();
        }

        PartModule findModuleByName(Part p, string name)
        {
            for (int i = 0; i < p.Modules.Count; i++)
                if (p.Modules[i].moduleName.Equals(name))
                    return p.Modules[i];
            return null;
        }

        // give back original gimbal response to engines before clearing engine list
        void return_gimbals()
        {            
            foreach (EngineMoment em in engines)
            {
                if (em.gimbal != null)
                    em.gimbal.Restore();
            }
        }

        [AutoGuiAttr("gimbal_spd_norm", false, "G4")]
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
                    IGimbal mg = engines[i].gimbal;
                    if (mg != null)
                    {
                        float norm_spd = mg.UseGimbalSpeed ?
                            mg.GimbalSpeed : float.PositiveInfinity;
                        if (norm_spd < gimbal_spd_norm)
                            gimbal_spd_norm = norm_spd;
                    }
                }
                // apply it
                for (int i = 0; i < engines.Count; i++)
                {
                    IGimbal mg = engines[i].gimbal;
                    if (mg != null)
                        mg.Modify(gimbal_spd_norm);
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
                ModuleEngines eng = engines[i].engine;
                if (eng.part.State == PartStates.DEAD || !eng.part.isAttached)
                {
                    moments_cycle_counter = 0;      // need to reform parts lists
                    continue;
                }
                Vector3 tpos = Vector3.zero;
                Vector3 tdir = Vector3.zero;
                Vector3 e_thrust = Vector3.zero;
                Vector3 e_potent = Vector3.zero;
                double e_max_thrust = 0.0;
                int tcount = eng.thrustTransforms.Count;
                for (int j = 0; j < tcount; j++)
                {
                    Transform trans = eng.thrustTransforms[j];
                    tpos = trans.position - CoM;
                    tdir = -trans.forward;
                    tpos = world_to_cntrl_part * tpos;
                    tdir = world_to_cntrl_part * tdir;
                    Vector3 torque_arm = -Vector3.Cross(tpos, tdir);       // minus because Unity's left-handed
                    float mult = eng.thrustTransformMultipliers[j];
                    if (eng.currentThrottle > 0.0f)
                    {
                        double nozzle_max_thrust = eng.finalThrust * mult / eng.currentThrottle;
                        e_max_thrust += nozzle_max_thrust;
                        if (engines[i].gimbal == null)
                            e_potent += torque_arm * (float)nozzle_max_thrust;
                        else
                        {
                            Quaternion temp_rot = trans.localRotation;
                            trans.localRotation = engines[i].gimbal.neutralLocalRotation(j);
                            Vector3 tdir_neutral = world_to_cntrl_part  * (-trans.forward);
                            trans.localRotation = temp_rot;
                            Vector3 torque_arm_neutral = -Vector3.Cross(tpos, tdir_neutral);
                            e_potent += torque_arm_neutral * (float)nozzle_max_thrust;
                        }
                    }
                    else
                        e_max_thrust += eng.maxThrust * mult;
                    engines_torque_principal += torque_arm * eng.finalThrust * mult;
                    engines_thrust_principal += eng.finalThrust * tdir * mult;
                    e_thrust += eng.finalThrust * mult * (-trans.forward);
                }
                abs_thrust += eng.finalThrust;
                engines[i].thrust = e_thrust;
                engines[i].potential_torque = e_potent;
                engines[i].torque_components = e_potent.normalized;
                engines[i].estimated_max_thrust = e_max_thrust * eng.thrustPercentage * 0.01f;
                engines[i].estimated_max_thrust_unlimited = e_max_thrust;
            }
        }

        float prev_abs_thrust = 0.0f;

        Vector3 prev_engines_torque, prev_engines_thrust;

        [AutoGuiAttr("e_torq_k0", false, "G3")]
        Vector3 engines_torque_k0;

        [AutoGuiAttr("e_torq_k1", false, "G3")]
        Vector3 engines_torque_k1;

        [AutoGuiAttr("e_thrust_k0", false, "G3")]
        Vector3 engines_thrust_k0;

        [AutoGuiAttr("e_thrust_k1", false, "G3")]
        Vector3 engines_thrust_k1;

        bool gimbals_tested = false;

        // Stupid linear authority of gimbals, verry approximate but simple and effective.
        // engines_torque = engines_torque_k0 + user_input * engines_torque_k1
        // engines_thrust = engines_thrust_k0 + user_input * engines_thrust_k1
        void get_gimbal_authority()
        {
            if (any_gimbals && (prev_abs_thrust != 0.0f) && (abs_thrust != 0.0f) && (gimbal_buf[0].Size >= 2))
            {
                Vector3 scaled_prev_torque = prev_engines_torque / prev_abs_thrust;
                Vector3 scaled_cur_torque = engines_torque_principal / abs_thrust;
                Vector3 scaled_prev_thrust = prev_engines_thrust / prev_abs_thrust;
                Vector3 scaled_cur_thrust = engines_thrust_principal / abs_thrust;
                for (int axis = 0; axis < 3; axis++)
                {
                    // prepare thrust index workaround
                    int t_axis = ROLL;
                    if (axis == PITCH)
                        t_axis = YAW;
                    if (axis == YAW)
                        t_axis = PITCH;

                    float cur_cntrl = gimbal_buf[axis].getLast();
                    float last_cntrl = gimbal_buf[axis].getFromTail(1);
                    if (Math.Abs(cur_cntrl - last_cntrl) > 0.02)            // only significant input signal changes are analyzed
                    {
                        gimbals_tested = true;
                        // torque
                        float k1 = (scaled_cur_torque[axis] - scaled_prev_torque[axis]) / (cur_cntrl - last_cntrl);
                        if (k1 < 0.0f)
                            k1 = 0.0f;
                        float k0 = scaled_cur_torque[axis] - cur_cntrl * k1;
                        engines_torque_k0[axis] = k0 * abs_thrust;
                        engines_torque_k1[axis] = k1 * abs_thrust;
                        // thrust
                        k1 = (scaled_cur_thrust[t_axis] - scaled_prev_thrust[t_axis]) / (cur_cntrl - last_cntrl);
                        k0 = scaled_cur_thrust[t_axis] - cur_cntrl * k1;
                        engines_thrust_k0[axis] = k0 * abs_thrust;
                        engines_thrust_k1[axis] = k1 * abs_thrust;
                    }
                    else
                    {
                        // torque
                        float k1 = 0.0f;
                        if (gimbals_tested)
                            k1 = engines_torque_k1[axis] / abs_thrust;
                        else
                            k1 = MOI[axis] / abs_thrust;
                        float k0 = scaled_cur_torque[axis] - cur_cntrl * k1;
                        engines_torque_k0[axis] = k0 * abs_thrust;
                        // thrust
                        k1 = engines_thrust_k1[axis] / abs_thrust;
                        k0 = scaled_cur_thrust[t_axis] - cur_cntrl * k1;
                        engines_thrust_k0[axis] = k0 * abs_thrust;
                    }
                }
            }
            else
            {
                engines_torque_k0 = engines_torque_principal;
                engines_torque_k1 = Vector3d.zero;
                engines_thrust_k0 = engines_thrust_principal;
                engines_thrust_k1 = Vector3d.zero;
            }
            prev_abs_thrust = abs_thrust;
            prev_engines_torque = engines_torque_principal;
            prev_engines_thrust = engines_thrust_principal;
        }
    }

}
