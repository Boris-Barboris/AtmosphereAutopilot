using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using UnityEngine;
using System.IO;

namespace AtmosphereAutopilot
{
	/// <summary>
	/// Class for medium-term flight model support. Deals with angular positions, 
	/// angle of attack, g-force.
	/// </summary>
	public sealed class MediumFlightModel : AutopilotModule, ISerializable
	{
		internal MediumFlightModel(Vessel v) :
			base(v, 8459383, "Medium-term flight model") { }

		protected override void OnActivate()
		{
			vessel.OnPreAutopilotUpdate += new FlightInputCallback(OnPreAutopilot);
		}

		protected override void OnDeactivate()
		{
			vessel.OnPreAutopilotUpdate -= new FlightInputCallback(OnPreAutopilot);
		}

		static readonly int BUFFER_SIZE = 10;

		/// <summary>
		/// Get angle of attack in radians.
		/// </summary>
		public double AoA { get { return aoa_pitch.getLast(); } }
        CircularBuffer<double> aoa_pitch = new CircularBuffer<double>(BUFFER_SIZE, true, 0.0);
		
		/// <summary>
		/// Get sideslip angle in radians.
		/// </summary>
		public double Sideslip { get { return aoa_yaw.getLast(); } }
        CircularBuffer<double> aoa_yaw = new CircularBuffer<double>(BUFFER_SIZE, true, 0.0);

		/// <summary>
		/// Get G-force.
		/// </summary>
		public double GForce { get { return g_force.getLast(); } }
        CircularBuffer<double> g_force = new CircularBuffer<double>(BUFFER_SIZE, true, 0.0);

		/// <summary>
		/// Get estimated structural-safe maximum angular speed for this vessel. Radians/second.
		/// </summary>
		public double MaxAngularSpeed(int axis)
		{
			return max_angular_v[axis];
		}

        [AutoGuiAttr("max angular v", false, null)]
        double[] max_angular_v = new double[3];

		/// <summary>
		/// Get estimated structural-safe maximum angular acceleration for this vessel. Radians/second^2.
		/// </summary>
		public double MaxAngularAcc(int axis)
		{
			return max_angular_dv[axis];
		}

        [AutoGuiAttr("max angular acc", false, null)]
        double[] max_angular_dv = new double[3];

		/// <summary>
		/// Maximum part offset from center of mass in rotation plane, specified by rotation axis. Meters.
		/// </summary>
		public double MaxOffsetFromCoM(int axis)
		{
			return lever_arm[axis];
		}

        [AutoGuiAttr("lever arms", false, null)]
        double[] lever_arm = new double[3];

        double max_lever_arm = 1.0;

		/// <summary>
		/// Maximum part offset from center of mass in all basis rotation planes. Meters.
		/// </summary>
		public double MaxOffsetFromCoMOmni { get { return max_lever_arm; } }

        int cycle_counter = 0;

		void OnPreAutopilot(FlightCtrlState state)	// update all flight characteristics
		{
			update_buffers();
            //update_frames();
            if (cycle_counter == 0)
                calculate_limits();
            cycle_counter = (cycle_counter + 1) % 500;
		}

		Vector3 up_srf_v;		// normalized velocity, projected to vessel up direction
		Vector3 fwd_srf_v;		// normalized velocity, projected to vessel forward direction
		Vector3 right_srf_v;	// normalized velocity, projected to vessel right direction

		void update_buffers()
		{
            // thx ferram
            up_srf_v = vessel.ReferenceTransform.up * Vector3.Dot(vessel.ReferenceTransform.up, vessel.srf_velocity.normalized);
            fwd_srf_v = vessel.ReferenceTransform.forward * Vector3.Dot(vessel.ReferenceTransform.forward, vessel.srf_velocity.normalized);
            right_srf_v = vessel.ReferenceTransform.right * Vector3.Dot(vessel.ReferenceTransform.right, vessel.srf_velocity.normalized);
            Vector3 tmpVec = up_srf_v + fwd_srf_v;
            double aoa_p = Math.Asin(Vector3.Dot(vessel.ReferenceTransform.forward.normalized, tmpVec.normalized));
            aoa_pitch.Put(aoa_p);
            tmpVec = up_srf_v + right_srf_v;
            double aoa_y = Math.Asin(Vector3.Dot(vessel.ReferenceTransform.right.normalized, tmpVec.normalized));
            aoa_yaw.Put(aoa_y);
            g_force.Put(vessel.geeForce_immediate);
		}

		//public Vector3d surf_ref_up, surf_ref_right, surf_ref_fwd;
		//public Vector3d surf_up, surf_right, surf_fwd;

		//void update_frames()
		//{
		//	// Surface reference frame basis vectors
		//	surf_ref_fwd = -(vessel.mainBody.transform.position - vessel.transform.position).normalized;
		//	surf_ref_up = vessel.mainBody.getRFrmVel(vessel.transform.position).normalized;
		//	surf_ref_right = Vector3d.Cross(surf_ref_up, -surf_ref_fwd).normalized;

		//	// Plane reference frame basis vectors in surface reference frame
		//	surf_up = shiftFrame(vessel.transform.up, surf_ref_right, surf_ref_up, surf_ref_fwd);
		//	surf_right = shiftFrame(vessel.transform.right, surf_ref_right, surf_ref_up, surf_ref_fwd);
		//	surf_fwd = shiftFrame(vessel.transform.forward, surf_ref_right, surf_ref_up, surf_ref_fwd);
		//}

		//Matrix rotateFrame(Vector3[] frame0, Vector3[] frame1)
		//{
		//	Matrix result = new Matrix(3, 3);
		//	for (int i = 0; i < 3; i++)
		//		for (int j = 0; j < 3; j++)
		//			result[i, j] = Vector3.Dot(frame1[i], frame0[j]);
		//	return result;
		//}

		//Vector3 shiftFrame(Vector3 vector, Vector3 frame_x, Vector3 frame_y, Vector3 frame_z)
		//{
		//	Vector3 result = new Vector3(
		//		Vector3.Dot(vector, frame_x),
		//		Vector3.Dot(vector, frame_y),
		//		Vector3.Dot(vector, frame_z));
		//	return result;
		//}

        [GlobalSerializable("max_part_acceleration")]
        [AutoGuiAttr("max part accel", true, "G8")]
        internal double max_part_acceleration = 30.0;		// approx 3g.

        void calculate_limits()
        {
            max_part_offset_from_com(lever_arm);
            max_lever_arm = lever_arm.Max();
            for (int i = 0; i < 3; i++)
            {
                max_angular_v[i] = Math.Sqrt(Math.Abs(max_part_acceleration) / lever_arm[i]);
                max_angular_dv[i] = max_part_acceleration / lever_arm[i];
            }
			// Very rough wing load approximation. We don't anything about wind area, so
			// let's just use vessel size, roughly estimated by lever arms.
			double wing_area_aprox = lever_arm[ROLL] * lever_arm[PITCH];
        }

        void max_part_offset_from_com(double[] offsets)
        {
            double max_o_pitch = 1.0;
            double max_o_roll = 1.0;
            double max_o_yaw = 1.0;
            Vector3 com = vessel.findWorldCenterOfMass();
            foreach (var part in vessel.Parts)
            {
                Vector3 part_v = part.transform.position - com;
                double o_pitch, o_roll, o_yaw;
                o_pitch = Vector3.Cross(part_v, vessel.transform.right).magnitude;
                o_roll = Vector3.Cross(part_v, vessel.transform.up).magnitude;
                o_yaw = Vector3.Cross(part_v, vessel.transform.forward).magnitude;
                if (o_pitch > max_o_pitch)
                    max_o_pitch = o_pitch;
                if (o_roll > max_o_roll)
                    max_o_roll = o_roll;
                if (o_yaw > max_o_yaw)
                    max_o_yaw = o_yaw;
            }
            offsets[PITCH] = max_o_pitch;
            offsets[ROLL] = max_o_roll;
            offsets[YAW] = max_o_yaw;
        }

        #region Serialization

        public override bool Deserialize()
        {
            return AutoSerialization.Deserialize(this, "MediumFlightModel",
                KSPUtil.ApplicationRootPath + "GameData/AtmosphereAutopilot/Global_settings.cfg",
                typeof(GlobalSerializable));
        }

        public override void Serialize()
        {
            AutoSerialization.Serialize(this, "MediumFlightModel",
                KSPUtil.ApplicationRootPath + "GameData/AtmosphereAutopilot/Global_settings.cfg",
                typeof(GlobalSerializable));
        }

        #endregion


        #region GUI

		protected override void _drawGUI(int id)
		{
			GUILayout.BeginVertical();
			GUILayout.Label("AoA = " + aoa_pitch.getLast().ToString("G8"), GUIStyles.labelStyleLeft);
            GUILayout.Label("Sideslip = " + aoa_yaw.getLast().ToString("G8"), GUIStyles.labelStyleLeft);
            GUILayout.Label("G-force = " + g_force.getLast().ToString("G8"), GUIStyles.labelStyleLeft);
            AutoGUI.AutoDrawObject(this);
			GUILayout.EndVertical();
			GUI.DragWindow();
        }

        #endregion
    }
}
