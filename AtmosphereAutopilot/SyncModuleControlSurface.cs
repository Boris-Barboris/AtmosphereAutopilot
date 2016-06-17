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
using UnityEngine;

namespace AtmosphereAutopilot
{

    /// <summary>
    /// Synchronised ModuleControlSurface realization, greatly simplifies control and flight model regression 
    /// by making all control surfaces move in one phase.
    /// </summary>
    public class SyncModuleControlSurface: ModuleControlSurface
    {
        public const float CSURF_SPD = 2.0f;

        protected float prev_pitch_action = 0.0f;
        protected float prev_roll_action = 0.0f;
        protected float prev_yaw_action = 0.0f;

        protected override void CtrlSurfaceUpdate(Vector3 vel)
        {
            if (vessel.transform == null)
                return;
            
            Vector3 world_com = vessel.CoM + vessel.rb_velocity * TimeWarp.fixedDeltaTime;
            float pitch_input = ignorePitch ? 0.0f : vessel.ctrlState.pitch;
            float roll_input = ignoreRoll ? 0.0f : vessel.ctrlState.roll;
            float yaw_input = ignoreYaw ? 0.0f : vessel.ctrlState.yaw;

            if (base.vessel.atmDensity == 0.0)
                pitch_input = roll_input = yaw_input = 0.0f;

            float spd_factor = TimeWarp.fixedDeltaTime * CSURF_SPD;
            float fwd_airstream_factor = Mathf.Sign(Vector3.Dot(vessel.ReferenceTransform.up, vessel.srf_velocity) + 0.1f);
            float exp_spd_factor = actuatorSpeed / actuatorSpeedNormScale * TimeWarp.fixedDeltaTime;

            if (this.deploy)
            {
                float target = this.deployInvert ? 1.0f : -1.0f;
                if (this.usesMirrorDeploy)
                    if (this.mirrorDeploy)
                        target *= -1.0f;
                if (!ignorePitch)
                    prev_pitch_action = target;
                if (!ignoreRoll)
                    prev_roll_action = target;
                if (!ignoreYaw)
                    prev_yaw_action = target;
                deflection = action = action + Common.Clampf(target - action, spd_factor);
                ctrlSurface.localRotation = Quaternion.AngleAxis(deflection * ctrlSurfaceRange * 0.01f * authorityLimiter, Vector3.right) * neutral;
            }
            else
            {
                if (!ignorePitch)
                {
                    float axis_factor = Vector3.Dot(vessel.ReferenceTransform.right, baseTransform.right) * fwd_airstream_factor;
                    float new_pitch_action = pitch_input * axis_factor * Math.Sign(Vector3.Dot(world_com - baseTransform.position, vessel.ReferenceTransform.up));
                    if (useExponentialSpeed)
                        prev_pitch_action = Mathf.Lerp(prev_pitch_action, new_pitch_action, exp_spd_factor);
                    else
                        prev_pitch_action = prev_pitch_action + Common.Clampf(new_pitch_action - prev_pitch_action, spd_factor * Math.Abs(axis_factor));
                }
                else
                    prev_pitch_action = 0.0f;

                if (!ignoreRoll)
                {
                    float axis_factor = Vector3.Dot(vessel.ReferenceTransform.up, baseTransform.up) * fwd_airstream_factor;
                    float new_roll_action = roll_input * axis_factor * Math.Sign(Vector3.Dot(vessel.ReferenceTransform.up,
                        Vector3.Cross(world_com - baseTransform.position, baseTransform.forward)));
                    if (useExponentialSpeed)
                        prev_roll_action = Mathf.Lerp(prev_roll_action, new_roll_action, exp_spd_factor);
                    else
                        prev_roll_action = prev_roll_action + Common.Clampf(new_roll_action - prev_roll_action, spd_factor * axis_factor);
                }
                else
                    prev_roll_action = 0.0f;

                if (!ignoreYaw)
                {
                    float axis_factor = Vector3.Dot(vessel.ReferenceTransform.forward, baseTransform.right) * fwd_airstream_factor;
                    float new_yaw_action = yaw_input * axis_factor * Math.Sign(Vector3.Dot(world_com - baseTransform.position, vessel.ReferenceTransform.up));
                    if (useExponentialSpeed)
                        prev_yaw_action = Mathf.Lerp(prev_yaw_action, new_yaw_action, exp_spd_factor);
                    else
                        prev_yaw_action = prev_yaw_action + Common.Clampf(new_yaw_action - prev_yaw_action, spd_factor * Math.Abs(axis_factor));
                }
                else
                    prev_yaw_action = 0.0f;

                deflection = action = Common.Clampf(prev_pitch_action + prev_roll_action + prev_yaw_action, 1.0f);
                ctrlSurface.localRotation = Quaternion.AngleAxis(deflection * ctrlSurfaceRange * 0.01f * authorityLimiter, Vector3.right) * neutral;
            }
        }


        public new void FixedUpdate()
        {
            if (HighLogic.LoadedSceneIsFlight)
            {
                if (base.part.Rigidbody != null && !base.part.ShieldedFromAirstream)
                {
                    Vector3 vel = base.part.Rigidbody.GetPointVelocity(this.baseTransform.position) + Krakensbane.GetFrameVelocityV3f();
                    double submergedPortion = base.part.submergedPortion;
                    double airPortion = 1.0 - submergedPortion;

                    this.Qlift = (base.part.dynamicPressurekPa * airPortion + base.part.submergedDynamicPressurekPa * submergedPortion * base.part.submergedLiftScalar) * 1000.0;
                    this.Qdrag = (base.part.dynamicPressurekPa * airPortion + base.part.submergedDynamicPressurekPa * submergedPortion * base.part.submergedDragScalar) * 1000.0;
                    base.SetupCoefficients(vel, out this.nVel, out this.liftVector, out this.liftDot, out this.absDot);
                    this.liftForce = base.GetLiftVector(this.liftVector, this.liftDot, this.absDot, this.Qlift, (float)base.part.machNumber) * (1f - this.ctrlSurfaceArea);
                    if (this.useInternalDragModel)
                    {
                        this.dragForce = base.GetDragVector(this.nVel, this.absDot, this.Qdrag) * (1f - this.ctrlSurfaceArea);
                    }
                    float num2 = this.liftDot;
                    float absDot = this.absDot;
                    if (this.ctrlSurface != null)
                    {
                        this.CtrlSurfaceUpdate(vel);
                        this.airflowIncidence = Quaternion.AngleAxis(this.ctrlSurfaceRange * this.deflection, this.baseTransform.rotation * Vector3.right);
                        this.liftVector = this.airflowIncidence * this.liftVector;
                        num2 = Vector3.Dot(this.nVel, this.liftVector);
                        absDot = Mathf.Abs(num2);
                    }
                    this.liftForce += base.GetLiftVector(this.liftVector, num2, absDot, this.Qlift, (float)base.part.machNumber) * this.ctrlSurfaceArea;
                    base.part.Rigidbody.AddForceAtPosition(this.liftForce, base.part.rb.worldCenterOfMass + base.part.partTransform.rotation * base.part.CoLOffset, ForceMode.Force);
                    if (this.useInternalDragModel)
                    {
                        this.dragForce += base.GetDragVector(this.nVel, absDot, this.Qdrag) * this.ctrlSurfaceArea;
                        base.part.Rigidbody.AddForceAtPosition(this.dragForce, base.part.rb.worldCenterOfMass + base.part.partTransform.rotation * base.part.CoPOffset, ForceMode.Force);
                    }
                    base.part.DragCubes.SetCubeWeight("neutral", 150.0f * 0.01f - Mathf.Abs(this.deflection * this.authorityLimiter * 0.01f));
                    base.part.DragCubes.SetCubeWeight("fullDeflectionPos", Mathf.Clamp01(this.deflection * this.authorityLimiter * 0.01f));
                    base.part.DragCubes.SetCubeWeight("fullDeflectionNeg", Mathf.Clamp01(-(this.deflection * this.authorityLimiter * 0.01f)));
                    base.UpdateAeroDisplay(Color.yellow);
                }
                else
                {
                    this.Qlift = (this.Qdrag = 0.0);
                    this.nVel = Vector3.zero;
                }
            }
        }

    }
}
