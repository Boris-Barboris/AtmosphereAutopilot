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
        //[AutoGuiAttr("angular_vel", false, "G8")]
        Vector3 angular_vel = Vector3.zero;

        [AutoGuiAttr("MOI", false, "G5")]
        public Vector3 MOI;

        public Vector3 AM;

        [AutoGuiAttr("CoM", false, "G5")]
        public Vector3 CoM;

        Vector3 partial_CoM;
        Vector3 cur_CoM;

        //[AutoGuiAttr("world_v", false, "G4")]
        public Vector3d surface_v;

        public double surface_v_magnitude;

        [AutoGuiAttr("Vessel mass", false, "G6")]
        public float sum_mass = 0.0f;

        // It's too expensive to iterate over all parts every physics frame, so we'll stick with 20 most massive
        const int PartsMax = 20;

        struct PartMass
        {
            public PartMass(Part p, float m) { part = p; mass = m; }
            public Part part;
            public float mass;
            public static int Comparison(PartMass x, PartMass y)
            {
                if (x.mass < y.mass)
                    return -1;
                else
                    if (x.mass == y.mass)
                        return 0;
                    else
                        return 1;
            }
        }

        List<PartMass> massive_parts = new List<PartMass>(PartsMax);
        Vector3 partial_MOI = Vector3.zero;
        Vector3 partial_AM = Vector3.zero;

        const int FullMomentFreq = 40;      // with standard 0.025 sec fixedDeltaTime it gives freq around 1 Hz
        int moments_cycle_counter = 0;

        [AutoGuiAttr("Reaction wheels", false, "G4")]
        public Vector3 reaction_torque = Vector3.zero;

        int prev_part_count = 0;

        void update_moments()
        {
            if (vessel.Parts.Count != prev_part_count)
                moments_cycle_counter = 0;
            prev_part_count = vessel.Parts.Count;
            if (moments_cycle_counter == 0)
            {
                check_csurfaces();
                get_moments(true);
                reaction_torque = get_sas_authority();
                get_rcs_characteristics();
                get_engines();
            }
            else
                get_moments(true);
            moments_cycle_counter = (moments_cycle_counter + 1) % FullMomentFreq;
        }
        
        [AutoGuiAttr("has csurf", false)]
        public bool HasControlSurfaces { get; private set; }

        void check_csurfaces()
        {
            HasControlSurfaces = vessel.Parts.Find(p => p.Modules.Contains(AtmosphereAutopilot.control_surface_module_type.Name)) != null;
        }

        Vector3 get_sas_authority()
        {
            Vector3 res = Vector3.zero;
            foreach (var rw in vessel.FindPartModulesImplementing<ModuleReactionWheel>())
                if (rw.isEnabled && rw.wheelState == ModuleReactionWheel.WheelState.Active && rw.operational)
                {
                    res.x += rw.PitchTorque;
                    res.y += rw.RollTorque;
                    res.z += rw.YawTorque;
                }
            return res;
        }

        Vector3 findPartialWorldCoM()
        {
            Vector3 result = Vector3.zero;
            float mass = 0.0f;
            foreach (var pm in massive_parts)
            {
                if (pm.part.State == PartStates.DEAD || !pm.part.isAttached)
                    continue;
                if (pm.part.rb != null)
                {
                    result += pm.part.rb.worldCenterOfMass * pm.part.rb.mass;
                    mass += pm.part.rb.mass;
                }
                else
                {
                    float pmass = pm.part.mass + pm.part.GetResourceMass();
                    mass += pmass;
                    result += (pm.part.partTransform.position + pm.part.partTransform.rotation * pm.part.CoMOffset) * pmass;
                }
            }
            if (mass > 0.0f)
                result /= mass;
            return result;
        }

        // Rotations of the currently controlling part of the vessel
        public Quaternion world_to_cntrl_part;
        public Quaternion cntrl_part_to_world;

        void get_moments(bool all_parts)
        {
            cntrl_part_to_world = vessel.ReferenceTransform.rotation;
            world_to_cntrl_part = cntrl_part_to_world.Inverse();            // from world to control part rotation
            CoM = vessel.findWorldCenterOfMass();
            if (all_parts)
            {
                MOI = Vector3d.zero;
                AM = Vector3d.zero;
                massive_parts.Clear();
                sum_mass = 0.0f;
                cur_CoM = CoM;
            }
            else
            {
                partial_MOI = Vector3.zero;
                partial_AM = Vector3.zero;
                cur_CoM = partial_CoM = findPartialWorldCoM();
            }

            int indexing = all_parts ? vessel.parts.Count : massive_parts.Count;

            // Get velocity
            surface_v = Vector3d.zero;
            Vector3d v_impulse = Vector3d.zero;
            double v_mass = 0.0;
            for (int pi = 0; pi < indexing; pi++)
            {
                Part part = all_parts ? vessel.parts[pi] : massive_parts[pi].part;
                if (part.physicalSignificance == Part.PhysicalSignificance.NONE)
                    continue;
                if (part.vessel != vessel || part.State == PartStates.DEAD)
                {
                    moments_cycle_counter = 0;      // iterating over old part list
                    continue;
                }
                float mass = 0.0f;
                if (part.rb != null)
                {
                    mass = part.rb.mass;
                    v_impulse += part.rb.velocity * mass;
                }
                else
                {
                    mass = part.mass + part.GetResourceMass();
                    v_impulse += part.vel * mass;
                }
                v_mass += mass;
            }
            surface_v = v_impulse / v_mass;

            // Get angular velocity
            for (int pi = 0; pi < indexing; pi++)
            {
                Part part = all_parts ? vessel.parts[pi] : massive_parts[pi].part;
                if (part.physicalSignificance == Part.PhysicalSignificance.NONE)
                    continue;
                if (part.vessel != vessel || part.State == PartStates.DEAD)
                {
                    moments_cycle_counter = 0;      // iterating over old part list
                    continue;
                }
                Quaternion part_to_cntrl = part.partTransform.rotation * world_to_cntrl_part;   // from part to root part rotation
                Vector3 moi = Vector3d.zero;
                Vector3 am = Vector3d.zero;
                float mass = 0.0f;
                if (part.rb != null)
                {
                    mass = part.rb.mass;
                    Vector3 world_pv = part.rb.worldCenterOfMass - cur_CoM;
                    Vector3 pv = world_to_cntrl_part * world_pv;
                    Vector3 impulse = mass * (world_to_cntrl_part * (part.rb.velocity - surface_v));
                    // from part.rb principal frame to root part rotation
                    Quaternion principal_to_cntrl = part.rb.inertiaTensorRotation * part_to_cntrl;
                    // MOI of part as offsetted material point
                    moi += mass * new Vector3(pv.y * pv.y + pv.z * pv.z, pv.x * pv.x + pv.z * pv.z, pv.x * pv.x + pv.y * pv.y);
                    // MOI of part as rigid body
                    Vector3 rotated_moi = get_rotated_moi(part.rb.inertiaTensor, principal_to_cntrl);
                    moi += rotated_moi;
                    // angular moment of part as offsetted material point
                    am += Vector3.Cross(pv, impulse);
                    // angular moment of part as rotating rigid body
                    am += Vector3.Scale(rotated_moi, world_to_cntrl_part * part.rb.angularVelocity);
                }
                else
                {
                    mass = part.mass + part.GetResourceMass();
                    Vector3 world_pv = part.partTransform.position + part.partTransform.rotation * part.CoMOffset - cur_CoM;
                    Vector3 pv = world_to_cntrl_part * world_pv;
                    Vector3 impulse = mass * (world_to_cntrl_part * (part.vel - surface_v));
                    // MOI of part as offsetted material point
                    moi += mass * new Vector3(pv.y * pv.y + pv.z * pv.z, pv.x * pv.x + pv.z * pv.z, pv.x * pv.x + pv.y * pv.y);
                    // angular moment of part as offsetted material point
                    am += Vector3.Cross(pv, impulse);
                }
                if (all_parts)
                {
                    massive_parts.Add(new PartMass(part, mass));
                    MOI += moi;
                    AM -= am;                   // minus because left handed Unity
                    sum_mass += mass;
                }
                else
                {
                    partial_MOI += moi;
                    partial_AM -= am;           // minus because left handed Unity
                }
            }
            if (all_parts)
            {
                /*
                massive_parts.Sort(PartMass.Comparison);
                if (massive_parts.Count > PartsMax)
                    massive_parts.RemoveRange(PartsMax, massive_parts.Count - PartsMax);
                */
                angular_vel = Common.divideVector(AM, MOI);
            }
            else
            {
                angular_vel = Common.divideVector(partial_AM, partial_MOI);
                surface_v -= Vector3.Cross(cur_CoM - CoM, cntrl_part_to_world * angular_vel);
            }
            angular_vel -= world_to_cntrl_part * vessel.mainBody.angularVelocity;     // remember that unity physics reference frame is rotating
            surface_v += Krakensbane.GetFrameVelocity();
            surface_v_magnitude = surface_v.magnitude;
        }

        static Vector3 get_rotated_moi(Vector3 inertia_tensor, Quaternion rotation)
        {
            Matrix4x4 inert_matrix = Matrix4x4.zero;
            for (int i = 0; i < 3; i++)
                inert_matrix[i, i] = inertia_tensor[i];
            Matrix4x4 rot_matrix = Common.rotationMatrix(rotation);
            Matrix4x4 new_inert = (rot_matrix * inert_matrix) * rot_matrix.transpose;
            return new Vector3(new_inert[0, 0], new_inert[1, 1], new_inert[2, 2]);
        }

        void update_velocity_acc()
        {
            for (int i = 0; i < 3; i++)
            {
                angular_v_buf[i].Put(angular_vel[i]);           // update angular velocity
                if (angular_v_buf[i].Size >= 2 && sequential_dt)
                    angular_acc_buf[i].Put(
                        Common.derivative1_short(
                            angular_v_buf[i].getFromTail(1),
                            angular_v_buf[i].getLast(),
                            TimeWarp.fixedDeltaTime));
            }
            // update surface velocity
        }

        public Vector3 up_srf_v;        // velocity, projected to vessel up direction
        public Vector3 fwd_srf_v;       // velocity, projected to vessel forward direction
        public Vector3 right_srf_v;     // velocity, projected to vessel right direction

        void update_aoa()
        {
            up_srf_v = Vector3.Project(surface_v, vessel.ReferenceTransform.up);
            fwd_srf_v = Vector3.Project(surface_v, vessel.ReferenceTransform.forward);
            right_srf_v = Vector3.Project(surface_v, vessel.ReferenceTransform.right);

            Vector3 projected_vel = up_srf_v + fwd_srf_v;
            if (projected_vel.sqrMagnitude > 1.0f)
            {
                float aoa_p = (float)Math.Asin(Common.Clampf(Vector3.Dot(vessel.ReferenceTransform.forward, projected_vel.normalized), 1.0f));
                if (Vector3.Dot(projected_vel, vessel.ReferenceTransform.up) < 0.0)
                    aoa_p = (float)Math.PI - aoa_p;
                aoa_buf[PITCH].Put(aoa_p);
            }
            else
                aoa_buf[PITCH].Put(0.0f);

            projected_vel = up_srf_v + right_srf_v;
            if (projected_vel.sqrMagnitude > 1.0f)
            {
                float aoa_y = (float)Math.Asin(Common.Clampf(Vector3.Dot(-vessel.ReferenceTransform.right, projected_vel.normalized), 1.0f));
                if (Vector3.Dot(projected_vel, vessel.ReferenceTransform.up) < 0.0)
                    aoa_y = (float)Math.PI - aoa_y;
                aoa_buf[YAW].Put(aoa_y);
            }
            else
                aoa_buf[YAW].Put(0.0f);

            projected_vel = right_srf_v + fwd_srf_v;
            if (projected_vel.sqrMagnitude > 1.0f)
            {
                float aoa_r = (float)Math.Asin(Common.Clampf(Vector3.Dot(vessel.ReferenceTransform.forward, projected_vel.normalized), 1.0f));
                if (Vector3.Dot(projected_vel, vessel.ReferenceTransform.right) < 0.0)
                    aoa_r = (float)Math.PI - aoa_r;
                aoa_buf[ROLL].Put(aoa_r);
            }
            else
                aoa_buf[ROLL].Put(0.0f);
        }

        void update_control(FlightCtrlState state)
        {
            for (int i = 0; i < 3; i++)
            {
                float raw_input = Common.Clampf(ControlUtils.getControlFromState(state, i), 1.0f);
                input_buf[i].Put(raw_input);
                if (HasControlSurfaces)
                {
                    float csurf_input = 0.0f;
                    if (AtmosphereAutopilot.AeroModel == AtmosphereAutopilot.AerodinamycsModel.Stock)
                    {
                        if (csurf_buf[i].Size >= 1 && sequential_dt)
                            csurf_input = stock_actuator_blend(csurf_buf[i].getLast(), raw_input);
                        else
                            csurf_input = raw_input;
                    }
                    else
                        if (AtmosphereAutopilot.AeroModel == AtmosphereAutopilot.AerodinamycsModel.FAR)
                        {
                            if (csurf_buf[i].Size >= 1 && sequential_dt)
                                csurf_input = far_exponential_blend(csurf_buf[i].getLast(), raw_input);
                            else
                                csurf_input = raw_input;
                        }
                    csurf_buf[i].Put(csurf_input);
                }
                else
                    csurf_buf[i].Put(0.0f);
                if (any_gimbals)
                {
                    if (gimbal_buf[i].Size >= 1 && sequential_dt && !float.IsPositiveInfinity(gimbal_spd_norm))
                        gimbal_buf[i].Put(exponential_blend(gimbal_buf[i].getLast(), raw_input, gimbal_spd_norm, 0.0f));
                    else
                        gimbal_buf[i].Put(raw_input);
                }
                else
                    gimbal_buf[i].Put(0.0f);
            }
        }

        public static bool far_blend_collapse(float prev, float desire)
        {
            return (Math.Abs((desire - prev) * 10.0f) < 0.1f);
        }

        public static float far_exponential_blend(float prev, float desire)
        {
            float error = desire - prev;
            if (Math.Abs(error * 10.0f) >= 0.1f)
            {
                return prev + Common.Clampf(error * TimeWarp.fixedDeltaTime / 0.25f, Math.Abs(0.6f * error));
            }
            else
                return desire;
        }

        public static float stock_actuator_blend(float prev, float desire)
        {
            float max_delta = TimeWarp.fixedDeltaTime * SyncModuleControlSurface.CSURF_SPD;
            return prev + Common.Clampf(desire - prev, max_delta);
        }

        public static float stock_blend_custom(float prev, float desire, float spd)
        {
            float max_delta = TimeWarp.fixedDeltaTime * spd;
            return prev + Common.Clampf(desire - prev, max_delta);
        }

        public static float exponential_blend(float prev, float desire, float spd, float snap_margin)
        {
            float error = desire - prev;
            if (Mathf.Abs(error) >= snap_margin)
            {
                return Mathf.Lerp(prev, desire, TimeWarp.fixedDeltaTime * spd);
            }
            else
                return desire;
        }

        List<ModuleRCS> rcs_thrusters;

        [AutoGuiAttr("RCS pos", false, "G4")]
        public Vector3 rcs_authority_pos;

        [AutoGuiAttr("RCS neg", false, "G4")]
        public Vector3 rcs_authority_neg;

        void get_rcs_characteristics()
        {
            rcs_thrusters = vessel.FindPartModulesImplementing<ModuleRCS>().Where(rcs => rcs.rcsEnabled && !rcs.part.ShieldedFromAirstream &&
                rcs.part.isControllable).ToList();
            if (vessel.ActionGroups[KSPActionGroup.RCS])
            {
                rcs_authority_pos = Vector3.zero;
                rcs_authority_neg = Vector3.zero;
                foreach (var rcsm in rcs_thrusters)
                {
                    for (int i = 0; i < rcsm.thrusterTransforms.Count; i++)
                    {
                        Transform t = rcsm.thrusterTransforms[i];
                        //float lever_k = rcsm.GetLeverDistance(-t.up, CoM);
                        Vector3 rv = world_to_cntrl_part * (t.position - CoM);
                        Vector3 rvn = rv.normalized;
                        Vector3 tup = world_to_cntrl_part * t.up;

                        // positive authority
                        float pitch_force = Mathf.Max(0.0f, Vector3.Dot(Vector3.Cross(new Vector3(1.0f, 0.0f, 0.0f), rvn), tup));
                        float roll_force = Mathf.Max(0.0f, Vector3.Dot(Vector3.Cross(new Vector3(0.0f, 1.0f, 0.0f), rvn), tup));
                        float yaw_force = Mathf.Max(0.0f, Vector3.Dot(Vector3.Cross(new Vector3(0.0f, 0.0f, 1.0f), rvn), tup));
                        float pitch_torque = Vector3.Cross(rv, tup * pitch_force).x;
                        float roll_torque = Vector3.Cross(rv, tup * roll_force).y;
                        float yaw_torque = Vector3.Cross(rv, tup * yaw_force).z;
                        rcs_authority_pos += new Vector3(pitch_torque, roll_torque, yaw_torque) * rcsm.thrusterPower * rcsm.thrustPercentage * 0.01f;

                        // negative authority
                        pitch_force = Mathf.Max(0.0f, Vector3.Dot(Vector3.Cross(new Vector3(-1.0f, 0.0f, 0.0f), rvn), tup));
                        roll_force = Mathf.Max(0.0f, Vector3.Dot(Vector3.Cross(new Vector3(0.0f, -1.0f, 0.0f), rvn), tup));
                        yaw_force = Mathf.Max(0.0f, Vector3.Dot(Vector3.Cross(new Vector3(0.0f, 0.0f, -1.0f), rvn), tup));
                        pitch_torque = Vector3.Cross(rv, tup * pitch_force).x;
                        roll_torque = Vector3.Cross(rv, tup * roll_force).y;
                        yaw_torque = Vector3.Cross(rv, tup * yaw_force).z;
                        rcs_authority_neg -= new Vector3(pitch_torque, roll_torque, yaw_torque) * rcsm.thrusterPower * rcsm.thrustPercentage * 0.01f;
                    }
                }
            }
            else
            {
                rcs_authority_pos = Vector3.zero;
                rcs_authority_neg = Vector3.zero;
            }
        }

        public double get_rcs_torque(int axis, float input)
        {
            if (input >= 0.0f)
                return input * rcs_authority_pos[axis];
            else
                return input * rcs_authority_neg[axis];
        }
    }
}
