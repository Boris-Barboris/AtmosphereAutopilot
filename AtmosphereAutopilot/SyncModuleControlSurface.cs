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
using UnityEngine;

namespace AtmosphereAutopilot
{
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
            Vector3 local_com = baseTransform.InverseTransformPoint(world_com);
            float pitch_input = ignorePitch ? 0.0f : vessel.ctrlState.pitch;
            float roll_input = ignoreRoll ? 0.0f : vessel.ctrlState.roll;
            float yaw_input = ignoreYaw ? 0.0f : vessel.ctrlState.yaw;

            if (this.deploy)
            {
                if (this.deployInvert)
                {
                    pitch_input = ignorePitch ? 0.0f : -1.0f;
                    roll_input = ignoreRoll ? 0.0f : -1.0f;
                    yaw_input = ignoreYaw ? 0.0f : -1.0f;
                }
                else
                {
                    pitch_input = ignorePitch ? 0.0f : 1.0f;
                    roll_input = ignoreRoll ? 0.0f : 1.0f;
                    yaw_input = ignoreYaw ? 0.0f : 1.0f;
                }
            }

            float spd_factor = TimeWarp.fixedDeltaTime * CSURF_SPD;

            if (!ignorePitch)
            {
                float axis_factor = Vector3.Dot(vessel.transform.right, baseTransform.right);
                float new_pitch_action = pitch_input * axis_factor * Math.Sign(local_com.y);
                prev_pitch_action = prev_pitch_action + Common.Clampf(new_pitch_action - prev_pitch_action, spd_factor * Math.Abs(axis_factor));
            }
            if (!ignoreRoll)
            {
                float axis_factor = Vector3.Dot(vessel.transform.up, baseTransform.up);
                float new_roll_action = roll_input * axis_factor * -Math.Sign(local_com.x);
                prev_roll_action = prev_roll_action + Common.Clampf(new_roll_action - prev_roll_action, spd_factor * Math.Abs(axis_factor));
            }
            if (!ignoreYaw)
            {
                float axis_factor = Vector3.Dot(vessel.transform.forward, baseTransform.right);
                float new_yaw_action = yaw_input * axis_factor * Math.Sign(local_com.y);
                prev_yaw_action = prev_yaw_action + Common.Clampf(new_yaw_action - prev_yaw_action, spd_factor * Math.Abs(axis_factor));
            }

            deflection = action = Common.Clampf(prev_pitch_action + prev_roll_action + prev_yaw_action, 1.0f);
            ctrlSurface.localRotation = Quaternion.AngleAxis(deflection * ctrlSurfaceRange, Vector3.right) * neutral;
        }
    }
}
