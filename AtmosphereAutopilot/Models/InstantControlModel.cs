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
	/// Short-term motion model. Is responsible for angular speed, angular acceleration, control signal
	/// history storage. Executes analysis of pitch, roll and yaw evolution and control authority.
	/// </summary>
	public sealed class InstantControlModel : AutopilotModule
	{
		internal InstantControlModel(Vessel v):
            base(v, 34278832, "Instant control model")
		{
			for (int i = 0; i < 3; i++)
			{
                input_buf[i] = new CircularBuffer<float>(BUFFER_SIZE, true, 0.0f);
                angular_v[i] = new CircularBuffer<float>(BUFFER_SIZE, true, 0.0f);
                angular_acc[i] = new CircularBuffer<double>(BUFFER_SIZE, true, 0.0f);
                aoa[i] = new CircularBuffer<float>(BUFFER_SIZE, true, 0.0f);
			}
		}

		protected override void OnActivate()
		{
			vessel.OnPreAutopilotUpdate += new FlightInputCallback(OnPreAutopilot);
			vessel.OnPostAutopilotUpdate += new FlightInputCallback(OnPostAutopilot);
		}

		protected override void OnDeactivate()
		{
			vessel.OnPreAutopilotUpdate -= new FlightInputCallback(OnPreAutopilot);
			vessel.OnPostAutopilotUpdate -= new FlightInputCallback(OnPostAutopilot);
		}

		static readonly int BUFFER_SIZE = 8;

		#region Exports

		/// <summary>
		/// Control signal history for pitch, roll or yaw. [-1.0, 1.0].
		/// </summary>
		public CircularBuffer<float> ControlInputHistory(int axis) { return input_buf[axis]; }
        /// <summary>
        /// Control signal for pitch, roll or yaw. [-1.0, 1.0].
        /// </summary>
        public float ControlInput(int axis) { return input_buf[axis].getLast(); }

        CircularBuffer<float>[] input_buf = new CircularBuffer<float>[3];

		/// <summary>
		/// Angular velocity history for pitch, roll or yaw. Radians per second.
		/// </summary>
        public CircularBuffer<float> AngularVelHistory(int axis) { return angular_v[axis]; }
        /// <summary>
        /// Angular velocity for pitch, roll or yaw. Radians per second.
        /// </summary>
        public float AngularVel(int axis) { return angular_v[axis].getLast(); }

        CircularBuffer<float>[] angular_v = new CircularBuffer<float>[3];

		/// <summary>
        /// Angular acceleration hitory for pitch, roll or yaw. Radians per second per second.
		/// </summary>
		public CircularBuffer<double> AngularAccHistory(int axis) { return angular_acc[axis]; }
        /// <summary>
        /// Angular acceleration for pitch, roll or yaw. Radians per second per second.
        /// </summary>
		public double AngularAcc(int axis) { return angular_acc[axis].getLast(); }

		CircularBuffer<double>[] angular_acc = new CircularBuffer<double>[3];

        /// <summary>
        /// Angle of attack hitory for pitch, roll or yaw. Radians.
        /// </summary>
        public CircularBuffer<float> AoAHistory(int axis) { return aoa[axis]; }
        /// <summary>
        /// Angle of attack for pitch, roll or yaw. Radians.
        /// </summary>
        public float AoA(int axis) { return aoa[axis].getLast(); }

        CircularBuffer<float>[] aoa = new CircularBuffer<float>[3];

		#endregion

		float dt = 1.0f;			// dt in this call

        void OnPostAutopilot(FlightCtrlState state)		// update control input
		{
			update_control(state);
		}

		void OnPreAutopilot(FlightCtrlState state)		// workhorse function
		{
			dt = TimeWarp.fixedDeltaTime;
            get_angular_motion();
			update_velocity_acc();
            update_aoa();
		}

		#region Angular_Update

        public Vector3 angular_vel = Vector3.zero;
        Vector3 MOI = Vector3.zero;
        Vector3 AngMoment = Vector3.zero;

        [AutoGuiAttr("mass", false, "G6")]
        float sum_mass = 0.0f;

        void get_angular_motion()
        {
            Quaternion world_to_root = vessel.rootPart.partTransform.rotation.Inverse();     // from world to root part rotation
            Vector3 moi = Vector3.zero;
            Vector3 am = Vector3.zero;
            Vector3 com = vessel.findWorldCenterOfMass();
            Vector3 world_v = vessel.rootPart.rb.velocity;
            sum_mass = 0.0f;
            foreach (var part in vessel.parts)
            {
                if (part.physicalSignificance == Part.PhysicalSignificance.NONE)
                    continue;
                Quaternion part_to_root = part.partTransform.rotation * world_to_root;   // from part to root part rotation
                if (part.rb != null)
                {
                    float mass = part.rb.mass;
                    sum_mass += mass;
                    Vector3 world_pv = part.rb.worldCenterOfMass - com;
                    Vector3 pv = world_to_root * world_pv;
                    Vector3 impulse = mass * (world_to_root * (part.rb.velocity - world_v));
                    // from part.rb principal frame to root part rotation
                    Quaternion principal_to_root = part.rb.inertiaTensorRotation * part_to_root;
                    // part as offsetted material point
                    moi += mass * new Vector3(pv.y * pv.y + pv.z * pv.z, pv.x * pv.x + pv.z * pv.z, pv.x * pv.x + pv.y * pv.y);
                    // part as rigid body moi over part CoM
                    Vector3 rotated_moi = get_rotated_moi(part.rb.inertiaTensor, principal_to_root);
                    moi += rotated_moi;
                    // part moment as offsetted material point
                    am += Vector3.Cross(pv, impulse);
                    // part moment as rotating rigid body over part CoM
                    am += Vector3.Scale(rotated_moi, world_to_root * part.rb.angularVelocity);
                }
                else
                {
                    /*
                    float mass = part.mass + part.GetResourceMass();
                    sum_mass += mass;
                    Vector3 world_pv = part.partTransform.position + part.partTransform.rotation * part.CoMOffset - com;
                    Vector3 pv = world_to_root * world_pv;
                    Vector3 world_impulse = mass * part.vel;
                    // part as offsetted material point
                    moi += mass * new Vector3(pv.y * pv.y + pv.z * pv.z, pv.x * pv.x + pv.z * pv.z, pv.x * pv.x + pv.y * pv.y);
                    am += world_to_root * Vector3.Cross(world_pv, world_impulse);
                    */
                }
            }
            MOI = moi;
            AngMoment = am;
            angular_vel = Common.divideVector(AngMoment, MOI);
        }

        [AutoGuiAttr("DEBUG Moments", true)]
        internal bool test_outputs = false;

        // Draw debug vectors here
        internal void Update()
        {
            if (test_outputs)
            {
                using (var writer = System.IO.File.CreateText(KSPUtil.ApplicationRootPath + "/Resources/moments.txt"))
                {
                    Quaternion world_to_root = vessel.rootPart.partTransform.rotation.Inverse();     // from world to root part rotation
                    writer.WriteLine("world_to_root = " + world_to_root.ToString("G6"));
                    Vector3 moi = Vector3.zero;
                    Vector3 am = Vector3.zero;
                    Vector3 com = vessel.findWorldCenterOfMass();
                    Vector3 world_v = vessel.rootPart.rb.velocity;
                    writer.WriteLine("com = " + com.ToString("G6"));
                    writer.WriteLine("world_v = " + world_v.ToString("G6"));
                    sum_mass = 0.0f;
                    foreach (var part in vessel.parts)
                    {
                        if (part.physicalSignificance == Part.PhysicalSignificance.NONE)
                            continue;
                        Quaternion part_to_root = part.partTransform.rotation * world_to_root;   // from part to root part rotation
                        if (part.rb != null)
                        {
                            writer.WriteLine("\r\nprocessing part " + part.partName);
                            float mass = part.rb.mass;
                            writer.WriteLine("mass = " + mass.ToString("G6"));
                            sum_mass += mass;
                            Vector3 world_pv = part.rb.worldCenterOfMass - com;
                            writer.WriteLine("part.rb.worldCenterOfMass = " + part.rb.worldCenterOfMass.ToString("G6"));
                            writer.WriteLine("world_pv = " + world_pv.ToString("G6"));
                            Vector3 pv = world_to_root * world_pv;
                            writer.WriteLine("pv = " + pv.ToString("G6"));
                            Vector3 impulse = mass * (world_to_root * (part.rb.velocity - world_v));
                            writer.WriteLine("part.rb.velocity = " + part.rb.velocity.ToString("G6"));
                            writer.WriteLine("impulse = " + impulse.ToString("G6"));
                            // from part.rb principal frame to root part rotation
                            Quaternion principal_to_root = part.rb.inertiaTensorRotation * part_to_root;
                            writer.WriteLine("part.rb.inertiaTensorRotation = " + part.rb.inertiaTensorRotation.ToString("G6"));
                            writer.WriteLine("principal_to_root = " + principal_to_root.ToString("G6"));
                            // part as offsetted material point
                            var dmoi = mass * new Vector3(pv.y * pv.y + pv.z * pv.z, pv.x * pv.x + pv.z * pv.z, pv.x * pv.x + pv.y * pv.y);
                            moi += dmoi;
                            writer.WriteLine("dmoi = " + dmoi.ToString("G6"));
                            // part as rigid body moi over part CoM
                            Vector3 rotated_moi = get_rotated_moi(part.rb.inertiaTensor, principal_to_root);
                            moi += rotated_moi;
                            writer.WriteLine("rotated_moi = " + rotated_moi.ToString("G6"));
                            // part moment as offsetted material point
                            var dam = Vector3.Cross(pv, impulse);
                            am += dam;
                            writer.WriteLine("dam = " + dam.ToString("G6"));
                            // part moment as rotating rigid body over part CoM
                            var self_am = world_to_root * part.rb.angularVelocity;
                            var d_self_am = Vector3.Scale(rotated_moi, self_am);
                            writer.WriteLine("self_am = " + self_am.ToString("G6"));
                            writer.WriteLine("d_self_am = " + d_self_am.ToString("G6"));
                            am += d_self_am;
                        }
                    }
                    MOI = moi;
                    AngMoment = am;
                    angular_vel = Common.divideVector(AngMoment, MOI);
                    writer.WriteLine();
                    writer.WriteLine("MOI = " + MOI.ToString("G6"));
                    writer.WriteLine("AngMoment = " + AngMoment.ToString("G6"));
                    writer.WriteLine("angular_vel = " + angular_vel.ToString("G6"));
                }
                test_outputs = false;
            }
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
				angular_v[i].Put(-vessel.angularVelocity[i]);	// update angular velocity. Minus for more meaningful
																// numbers (pitch up is positive)
                if (angular_v[i].Size >= 2)
					angular_acc[i].Put(
						Common.derivative1_short(
							angular_v[i].getFromTail(1),
							angular_v[i].getLast(),
							dt));
			}
		}

        public Vector3 up_srf_v;		// velocity, projected to vessel up direction
        public Vector3 fwd_srf_v;		// velocity, projected to vessel forward direction
        public Vector3 right_srf_v;		// velocity, projected to vessel right direction

        void update_aoa()
        {
            // thx ferram
            up_srf_v = vessel.ReferenceTransform.up * Vector3.Dot(vessel.ReferenceTransform.up, vessel.srf_velocity);
            fwd_srf_v = vessel.ReferenceTransform.forward * Vector3.Dot(vessel.ReferenceTransform.forward, vessel.srf_velocity);
            right_srf_v = vessel.ReferenceTransform.right * Vector3.Dot(vessel.ReferenceTransform.right, vessel.srf_velocity);

			Vector3 project = up_srf_v + fwd_srf_v;
			if (project.sqrMagnitude > 1.0f)
			{
				float aoa_p = (float)Math.Asin(Common.Clampf(Vector3.Dot(vessel.ReferenceTransform.forward.normalized, project.normalized), 1.0f));
				if (Vector3.Dot(project, vessel.ReferenceTransform.up) < 0.0)
					aoa_p = (float)Math.PI - aoa_p;
				aoa[PITCH].Put(aoa_p);
			}
			else
				aoa[PITCH].Put(0.0f);
			
			project = up_srf_v + right_srf_v;
			if (project.sqrMagnitude > 1.0f)
			{
				float aoa_y = (float)Math.Asin(Common.Clampf(Vector3.Dot(vessel.ReferenceTransform.right.normalized, project.normalized), 1.0f));
				if (Vector3.Dot(project, vessel.ReferenceTransform.up) < 0.0)
					aoa_y = (float)Math.PI - aoa_y;
				aoa[YAW].Put(aoa_y);
			}
			else
				aoa[YAW].Put(0.0f);

			project = right_srf_v + fwd_srf_v;
			if (project.sqrMagnitude > 1.0f)
			{
				float aoa_r = (float)Math.Asin(Common.Clampf(Vector3.Dot(vessel.ReferenceTransform.forward.normalized, project.normalized), 1.0f));
				if (Vector3.Dot(project, vessel.ReferenceTransform.right) < 0.0)
					aoa_r = (float)Math.PI - aoa_r;
				aoa[ROLL].Put(aoa_r);
			}
			else
				aoa[ROLL].Put(0.0f);
        }

		void update_control(FlightCtrlState state)
		{
			for (int i = 0; i < 3; i++)
				input_buf[i].Put(Common.Clampf(ControlUtils.getControlFromState(state, i), 1.0f));
		}

		#endregion

		#region Model_Identification

		

		#endregion

		#region Serialization

		public override bool Deserialize()
        {
            return AutoSerialization.Deserialize(this, "InstantControlModel",
                KSPUtil.ApplicationRootPath + "GameData/AtmosphereAutopilot/Global_settings.cfg",
                typeof(GlobalSerializable));
        }

        public override void Serialize()
        {
            AutoSerialization.Serialize(this, "InstantControlModel",
                KSPUtil.ApplicationRootPath + "GameData/AtmosphereAutopilot/Global_settings.cfg",
                typeof(GlobalSerializable));
        }

        #endregion

        #region GUI

		static readonly string[] axis_names = { "pitch", "roll", "yaw" };
        static readonly float rad_to_degree = (float)(180.0 / Math.PI);

		protected override void _drawGUI(int id)
		{
			GUILayout.BeginVertical();
			for (int i = 0; i < 3; i++)
			{
				GUILayout.Label(axis_names[i] + " ang vel = " + angular_v[i].getLast().ToString("G8"), GUIStyles.labelStyleLeft);
                GUILayout.Label(axis_names[i] + " ang acc = " + angular_acc[i].getLast().ToString("G8"), GUIStyles.labelStyleLeft);
                GUILayout.Label(axis_names[i] + " AoA = " + (aoa[i].getLast() * rad_to_degree).ToString("G8"), GUIStyles.labelStyleLeft);
                GUILayout.Label(axis_names[i] + " MOI = " + MOI[i].ToString("G8"), GUIStyles.labelStyleLeft);
                GUILayout.Label(axis_names[i] + " AngMoment = " + AngMoment[i].ToString("G8"), GUIStyles.labelStyleLeft);
                GUILayout.Label(axis_names[i] + " NewVel = " + angular_vel[i].ToString("G8"), GUIStyles.labelStyleLeft);
				GUILayout.Space(5);
			}
            AutoGUI.AutoDrawObject(this);
			GUILayout.EndVertical();
			GUI.DragWindow();
        }

        #endregion
    }
}
