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

	/// <summary>
	/// Short-term motion model. Is responsible for angular velocity, angular acceleration, control signal and 
	/// angle of attack evaluation and storage. Executes analysis of pitch, roll and yaw evolution and control authority.
	/// </summary>
	public sealed class InstantControlModel : AutopilotModule
	{
		internal InstantControlModel(Vessel v):
            base(v, 34278832, "Instant control model")
		{
			for (int i = 0; i < 3; i++)
			{
                input_buf[i] = new CircularBuffer<float>(BUFFER_SIZE, true);
                csurf_buf[i] = new CircularBuffer<float>(BUFFER_SIZE, true);
                angular_v_buf[i] = new CircularBuffer<float>(BUFFER_SIZE, true);
                angular_acc_buf[i] = new CircularBuffer<double>(BUFFER_SIZE, true);
                aoa_buf[i] = new CircularBuffer<float>(BUFFER_SIZE, true);
			}
            initialize_ann_tainers();
		}

		protected override void OnActivate()
		{
			vessel.OnPreAutopilotUpdate += new FlightInputCallback(OnPreAutopilot);
			vessel.OnPostAutopilotUpdate += new FlightInputCallback(OnPostAutopilot);
            AtmosphereAutopilot.Instance.BackgroundThread.add_func(train_pitch_ann);
            AtmosphereAutopilot.Instance.BackgroundThread.add_func(train_roll_ann);
            AtmosphereAutopilot.Instance.BackgroundThread.add_func(train_yaw_ann);
            AtmosphereAutopilot.Instance.BackgroundThread.Start();
		}

		protected override void OnDeactivate()
		{
			vessel.OnPreAutopilotUpdate -= new FlightInputCallback(OnPreAutopilot);
			vessel.OnPostAutopilotUpdate -= new FlightInputCallback(OnPostAutopilot);
            AtmosphereAutopilot.Instance.BackgroundThread.remove_func(train_pitch_ann);
            AtmosphereAutopilot.Instance.BackgroundThread.remove_func(train_roll_ann);
            AtmosphereAutopilot.Instance.BackgroundThread.remove_func(train_yaw_ann);
		}

		static readonly int BUFFER_SIZE = 8;



        #region Buffers

        CircularBuffer<float>[] input_buf = new CircularBuffer<float>[3];

        CircularBuffer<float>[] csurf_buf = new CircularBuffer<float>[3];   // True control surface rotation values

        CircularBuffer<float>[] angular_v_buf = new CircularBuffer<float>[3];

        CircularBuffer<double>[] angular_acc_buf = new CircularBuffer<double>[3];

        CircularBuffer<float>[] aoa_buf = new CircularBuffer<float>[3];

        #endregion



        #region BufferExports

        /// <summary>
		/// Control signal history for pitch, roll or yaw. [-1.0, 1.0].
		/// </summary>
		public CircularBuffer<float> ControlInputHistory(int axis) { return input_buf[axis]; }
        /// <summary>
        /// Control signal for pitch, roll or yaw. [-1.0, 1.0].
        /// </summary>
        public float ControlInput(int axis) { return input_buf[axis].getLast(); }

        /// <summary>
        /// Control surface position for pitch, roll or yaw. [-1.0, 1.0].
        /// </summary>
        public float ControlSurfPos(int axis) { return csurf_buf[axis].getLast(); }

		/// <summary>
		/// Angular velocity history for pitch, roll or yaw. Radians per second.
		/// </summary>
        public CircularBuffer<float> AngularVelHistory(int axis) { return angular_v_buf[axis]; }
        /// <summary>
        /// Angular velocity for pitch, roll or yaw. Radians per second.
        /// </summary>
        public float AngularVel(int axis) { return angular_v_buf[axis].getLast(); }

		/// <summary>
        /// Angular acceleration hitory for pitch, roll or yaw. Radians per second per second.
		/// </summary>
		public CircularBuffer<double> AngularAccHistory(int axis) { return angular_acc_buf[axis]; }
        /// <summary>
        /// Angular acceleration for pitch, roll or yaw. Radians per second per second.
        /// </summary>
		public double AngularAcc(int axis) { return angular_acc_buf[axis].getLast(); }

        /// <summary>
        /// Angle of attack hitory for pitch, roll or yaw. Radians.
        /// </summary>
        public CircularBuffer<float> AoAHistory(int axis) { return aoa_buf[axis]; }
        /// <summary>
        /// Angle of attack for pitch, roll or yaw. Radians.
        /// </summary>
        public float AoA(int axis) { return aoa_buf[axis].getLast(); }

		#endregion



        void OnPostAutopilot(FlightCtrlState state)		// update control input
		{
			update_control(state);
		}

		void OnPreAutopilot(FlightCtrlState state)		// workhorse function
		{
            update_moments();
			update_velocity_acc();
            update_aoa();
            if (angular_acc_buf[0].Size > 0)
            {
                update_model_acc();
                update_training_inputs();
                update_cpu();
            }
		}



		#region RotationUpdate

        Vector3 angular_vel = Vector3.zero;
        Vector3 MOI = Vector3.zero;
        Vector3 AM = Vector3.zero;
        Vector3 CoM = Vector3.zero;

        [AutoGuiAttr("mass", false, "G6")]
        float sum_mass = 0.0f;

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

        const int FullMomentFreq = 40;
        int cycle_counter = 0;

        void update_moments()
        {
            if (cycle_counter == 0)
                get_moments(true);
            else
                get_moments(false);
            cycle_counter = (cycle_counter + 1) % FullMomentFreq;
        }

        void get_moments(bool all_parts)
        {
            Quaternion world_to_root = vessel.rootPart.partTransform.rotation.Inverse();     // from world to root part rotation
            CoM = vessel.findWorldCenterOfMass();
            Vector3 world_v = vessel.rootPart.rb.velocity;
            if (all_parts)
            {
                MOI = Vector3.zero;
                AM = Vector3.zero;
                massive_parts.Clear();
                sum_mass = 0.0f;
            }
            partial_MOI = Vector3.zero;
            partial_AM = Vector3.zero;
            int indexing = all_parts ? vessel.parts.Count : massive_parts.Count;
            for (int pi = 0; pi < indexing; pi++)
            {
                Part part = all_parts ? vessel.parts[pi] : massive_parts[pi].part;
                if (part.physicalSignificance == Part.PhysicalSignificance.NONE)
                    continue;
                if (!part.isAttached || part.State == PartStates.DEAD)
                {
                    cycle_counter = 0;      // iterating over old part list
                    continue;
                }
                Quaternion part_to_root = part.partTransform.rotation * world_to_root;   // from part to root part rotation
                Vector3 moi = Vector3.zero;
                Vector3 am = Vector3.zero;
                float mass = 0.0f;
                if (part.rb != null)
                {
                    mass = part.rb.mass;
                    Vector3 world_pv = part.rb.worldCenterOfMass - CoM;
                    Vector3 pv = world_to_root * world_pv;
                    Vector3 impulse = mass * (world_to_root * (part.rb.velocity - world_v));
                    // from part.rb principal frame to root part rotation
                    Quaternion principal_to_root = part.rb.inertiaTensorRotation * part_to_root;
                    // MOI of part as offsetted material point
                    moi += mass * new Vector3(pv.y * pv.y + pv.z * pv.z, pv.x * pv.x + pv.z * pv.z, pv.x * pv.x + pv.y * pv.y);
                    // MOI of part as rigid body
                    Vector3 rotated_moi = get_rotated_moi(part.rb.inertiaTensor, principal_to_root);
                    moi += rotated_moi;
                    // angular moment of part as offsetted material point
                    am += Vector3.Cross(pv, impulse);
                    // angular moment of part as rotating rigid body
                    am += Vector3.Scale(rotated_moi, world_to_root * part.rb.angularVelocity);
                }
                else
                {
                    mass = part.mass + part.GetResourceMass();
                    Vector3 world_pv = part.partTransform.position + part.partTransform.rotation * part.CoMOffset - CoM;
                    Vector3 pv = world_to_root * world_pv;
                    Vector3 impulse = mass * (world_to_root * (part.vel - world_v));
                    // MOI of part as offsetted material point
                    moi += mass * new Vector3(pv.y * pv.y + pv.z * pv.z, pv.x * pv.x + pv.z * pv.z, pv.x * pv.x + pv.y * pv.y);
                    // angular moment of part as offsetted material point
                    am += Vector3.Cross(pv, impulse);
                }
                if (all_parts)
                {
                    massive_parts.Add(new PartMass(part, mass));
                    MOI += moi;
                    AM += am;
                    sum_mass += mass;
                }
                partial_MOI += moi;
                partial_AM += am;
            }
            if (all_parts)
            {
                massive_parts.Sort(PartMass.Comparison);
                if (massive_parts.Count > PartsMax)
                    massive_parts.RemoveRange(PartsMax, massive_parts.Count - 1);
            }
            angular_vel = -Common.divideVector(partial_AM, partial_MOI);
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
                    AM = am;
                    angular_vel = Common.divideVector(AM, MOI);
                    writer.WriteLine();
                    writer.WriteLine("MOI = " + MOI.ToString("G6"));
                    writer.WriteLine("AngMoment = " + AM.ToString("G6"));
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
                angular_v_buf[i].Put(angular_vel[i]);	        // update angular velocity
                if (angular_v_buf[i].Size >= 2)
					angular_acc_buf[i].Put(
						Common.derivative1_short(
							angular_v_buf[i].getFromTail(1),
							angular_v_buf[i].getLast(),
							TimeWarp.fixedDeltaTime));
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
				aoa_buf[PITCH].Put(aoa_p);
			}
			else
				aoa_buf[PITCH].Put(0.0f);
			
			project = up_srf_v + right_srf_v;
			if (project.sqrMagnitude > 1.0f)
			{
				float aoa_y = (float)Math.Asin(Common.Clampf(Vector3.Dot(vessel.ReferenceTransform.right.normalized, project.normalized), 1.0f));
				if (Vector3.Dot(project, vessel.ReferenceTransform.up) < 0.0)
					aoa_y = (float)Math.PI - aoa_y;
				aoa_buf[YAW].Put(aoa_y);
			}
			else
				aoa_buf[YAW].Put(0.0f);

			project = right_srf_v + fwd_srf_v;
			if (project.sqrMagnitude > 1.0f)
			{
				float aoa_r = (float)Math.Asin(Common.Clampf(Vector3.Dot(vessel.ReferenceTransform.forward.normalized, project.normalized), 1.0f));
				if (Vector3.Dot(project, vessel.ReferenceTransform.right) < 0.0)
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
                if (csurf_buf[i].Size >= 1)
                    csurf_buf[i].Put(far_exponential_blend(csurf_buf[i].getLast(), raw_input));
                else
                    csurf_buf[i].Put(raw_input);
            }
		}

        float far_exponential_blend(float prev, float desire)
        {
            float error = desire - prev;
            if (Math.Abs(error * 20.0f) > 0.1f)
            {
                return prev + Common.Clampf(error * TimeWarp.fixedDeltaTime / 0.25f, Math.Abs(0.6f * error));
            }
            else
                return desire;
        }

		#endregion



		#region ModelIdentification

        public Vector3d model_acc = Vector3d.zero;

        SimpleAnn pitch_ann = new SimpleAnn(4, 2);
        SimpleAnn roll_ann = new SimpleAnn(4, 3);
        SimpleAnn yaw_ann = new SimpleAnn(4, 2);

        const int IMM_BUF_SIZE = 20;

        OnlineAnnTrainer pitch_trainer, roll_trainer, yaw_trainer;
        OnlineAnnTrainer[] trainers = new OnlineAnnTrainer[3];

        void initialize_ann_tainers()
        {
            pitch_trainer = new OnlineAnnTrainer(pitch_ann, IMM_BUF_SIZE, new int[] { 21, 21 },
                new double[] { -1.0, -0.3 }, new double[] { 1.0, 0.3 }, pitch_input_method, pitch_output_method);
            trainers[0] = pitch_trainer;
            roll_trainer = new OnlineAnnTrainer(roll_ann, IMM_BUF_SIZE, new int[] { 11, 11, 11 },
                new double[] { -1.0, -1.0, -2.0 }, new double[] { 1.0, 1.0, 2.0 }, roll_input_method, roll_output_method);
            trainers[1] = roll_trainer;
            yaw_trainer = new OnlineAnnTrainer(yaw_ann, IMM_BUF_SIZE, new int[] { 11, 11 },
                new double[] { -1.0, -0.3 }, new double[] { 1.0, 0.3 }, yaw_input_method, yaw_output_method);
            trainers[2] = yaw_trainer;
        }

        double dyn_pressure = 1.0;

        void pitch_input_method(Vector v)
        {
            v[0] = csurf_buf[PITCH].getLast();
            v[1] = aoa_buf[PITCH].getFromTail(1);
        }

        double pitch_output_method()
        {
            return angular_acc_buf[PITCH].getLast() / dyn_pressure * 1e4;
        }

        void roll_input_method(Vector v)
        {
            v[0] = csurf_buf[ROLL].getLast();
            v[1] = csurf_buf[YAW].getLast();
            v[2] = angular_v_buf[ROLL].getFromTail(1) / Math.PI / 2.0;
        }

        double roll_output_method()
        {
            return angular_acc_buf[ROLL].getLast() / dyn_pressure * 1e4;
        }

        void yaw_input_method(Vector v)
        {
            v[0] = csurf_buf[YAW].getLast();
            v[1] = aoa_buf[YAW].getFromTail(1);
        }

        double yaw_output_method()
        {
            return angular_acc_buf[YAW].getLast() / dyn_pressure * 1e4;
        }

        // Training uinputs updating
        int cur_time = 0;
        int last_updated = 0;
        const int time_reset = int.MaxValue / 2;

        void update_training_inputs()
        {
            cur_time += (int)Math.Round(Time.fixedDeltaTime * 100.0f);
            if (cur_time >= time_reset)
            {
                cur_time -= time_reset;
                last_updated -= time_reset;
            }
            if (!vessel.LandedOrSplashed)
            {
                dyn_pressure = vessel.atmDensity * (up_srf_v + fwd_srf_v).sqrMagnitude;
                if (dyn_pressure < 10.0)
                    return;
                int time_elapsed = cur_time - last_updated;
                last_updated = cur_time;
                pitch_trainer.UpdateState(time_elapsed);
                roll_trainer.UpdateState(time_elapsed);
                yaw_trainer.UpdateState(time_elapsed);
            }
        }

        // Training methods
        [AutoGuiAttr("pitch_cpu", false)]
        int pitch_cpu = 0;

        [AutoGuiAttr("roll_cpu", false)]
        int roll_cpu = 0;

        [AutoGuiAttr("yaw_cpu", false)]
        int yaw_cpu = 0;

        [AutoGuiAttr("CPU per update", true)]
        int CPU_TIME_FOR_FIXEDUPDATE = 4;

        const int DESCEND_COST = 10;

        void update_cpu()
        {
            Interlocked.Add(ref pitch_cpu, CPU_TIME_FOR_FIXEDUPDATE);
            Interlocked.Add(ref roll_cpu, CPU_TIME_FOR_FIXEDUPDATE);
            Interlocked.Add(ref yaw_cpu, CPU_TIME_FOR_FIXEDUPDATE);
        }

        bool train_pitch_ann()
        {
            if (pitch_cpu >= DESCEND_COST)
            {
                Interlocked.Add(ref pitch_cpu, -DESCEND_COST);
                pitch_trainer.Train();
                return true;
            }
            return false;
        }

        bool train_roll_ann()
        {
            if (roll_cpu >= DESCEND_COST)
            {
                Interlocked.Add(ref roll_cpu, -DESCEND_COST);
                roll_trainer.Train();
                return true;
            }
            return false;
        }

        bool train_yaw_ann()
        {
            if (yaw_cpu >= DESCEND_COST)
            {
                Interlocked.Add(ref yaw_cpu, -DESCEND_COST);
                yaw_trainer.Train();
                return true;
            }
            return false;
        }

        // Ann evaluation
        Vector temp_v2 = new Vector(2);
        Vector temp_v3 = new Vector(3);
        void update_model_acc()
        {
            pitch_input_method(temp_v2);
            model_acc[0] = pitch_ann.eval(temp_v2) / 1e4 * dyn_pressure;
            roll_input_method(temp_v3);
            model_acc[1] = roll_ann.eval(temp_v3) / 1e4 * dyn_pressure;
            yaw_input_method(temp_v2);
            model_acc[2] = yaw_ann.eval(temp_v2) / 1e4 * dyn_pressure;
        }

		#endregion



		#region Serialization

        protected override void OnDeserialize(ConfigNode node, Type attribute_type)
        {
            if (attribute_type == typeof(VesselSerializable))
            {
                SimpleAnn pann = SimpleAnn.DeserializeFromNode(node, "pitch_ann");
                if (pann != null)
                    pitch_ann = pann;
                SimpleAnn rann = SimpleAnn.DeserializeFromNode(node, "roll_ann");
                if (rann != null)
                    roll_ann = rann;
                SimpleAnn yann = SimpleAnn.DeserializeFromNode(node, "yaw_ann");
                if (yann != null)
                    yaw_ann = yann;
            }
        }

        protected override void OnSerialize(ConfigNode node, Type attribute_type)
        {
            if (attribute_type == typeof(VesselSerializable))
            {
                pitch_ann.SerializeToNode(node, "pitch_ann");
                roll_ann.SerializeToNode(node, "roll_ann");
                yaw_ann.SerializeToNode(node, "yaw_ann");
            }
        }

        #endregion

        #region GUI

		static readonly string[] axis_names = { "pitch", "roll", "yaw" };
        const float rad2degree = (float)(180.0 / Math.PI);

		protected override void _drawGUI(int id)
		{
			GUILayout.BeginVertical();
			for (int i = 0; i < 3; i++)
			{
                GUILayout.Label("=======" + axis_names[i] + "=======");
				GUILayout.Label("ang vel = " + angular_v_buf[i].getLast().ToString("G8"), GUIStyles.labelStyleLeft);
                GUILayout.Label("ang acc = " + angular_acc_buf[i].getLast().ToString("G8"), GUIStyles.labelStyleLeft);
                GUILayout.Label("AoA = " + (aoa_buf[i].getLast() * rad2degree).ToString("G8"), GUIStyles.labelStyleLeft);
                GUILayout.Label("MOI = " + MOI[i].ToString("G8"), GUIStyles.labelStyleLeft);
                GUILayout.Label("AngMoment = " + AM[i].ToString("G8"), GUIStyles.labelStyleLeft);
                AutoGUI.AutoDrawObject(trainers[i]);
			}
            AutoGUI.AutoDrawObject(this);
			GUILayout.EndVertical();
			GUI.DragWindow();
        }

        #endregion
    }
}
