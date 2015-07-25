/*
Copyright 2015, Boris-Barboris

This file is part of Atmosphere Autopilot.
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
            AtmosphereAutopilot.Instance.BackgroundThread.add_func(train_pitch_lift);
            AtmosphereAutopilot.Instance.BackgroundThread.add_func(train_yaw_lift);
            AtmosphereAutopilot.Instance.BackgroundThread.Start();
		}

		protected override void OnDeactivate()
		{
			vessel.OnPreAutopilotUpdate -= new FlightInputCallback(OnPreAutopilot);
			vessel.OnPostAutopilotUpdate -= new FlightInputCallback(OnPostAutopilot);
            AtmosphereAutopilot.Instance.BackgroundThread.remove_func(train_pitch_ann);
            AtmosphereAutopilot.Instance.BackgroundThread.remove_func(train_roll_ann);
            AtmosphereAutopilot.Instance.BackgroundThread.remove_func(train_yaw_ann);
            AtmosphereAutopilot.Instance.BackgroundThread.remove_func(train_pitch_lift);
            AtmosphereAutopilot.Instance.BackgroundThread.remove_func(train_yaw_lift);
            return_gimbals();
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
        /// Lagged control surface position history for pitch, roll or yaw. [-1.0, 1.0].
        /// </summary>
        public CircularBuffer<float> ControlSurfPosHistory(int axis) { return csurf_buf[axis]; }
        /// <summary>
        /// Lagged control surface position for pitch, roll or yaw. [-1.0, 1.0].
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
            update_engine_moments();
            get_gimbal_authority();
            update_dynamics();
            if (angular_acc_buf[0].Size > 0)
            {
                update_model_acc();
                update_training_inputs();
                update_cpu();
            }
            update_pitch_rot_model();
            update_yaw_rot_model();
		}



		#region RotationUpdate

        Vector3 angular_vel = Vector3.zero;
        
        [AutoGuiAttr("MOI", false, "G6")]
        public Vector3 MOI = Vector3.zero;

        public Vector3 AM = Vector3.zero;

        //[AutoGuiAttr("CoM", false, "G6")]
        public Vector3 CoM = Vector3.zero;

        Vector3 partial_CoM = Vector3.zero;

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

        const int FullMomentFreq = 160;      // with standard 0.025 sec fixedDeltaTime it gives freq around 0.25 Hz
        int moments_cycle_counter = 0;

        [AutoGuiAttr("Reaction wheels", false, "G6")]
        public Vector3 reaction_torque = Vector3.zero;

        void update_moments()
        {
            if (moments_cycle_counter == 0)
            {
                get_moments(true);
                reaction_torque = get_sas_authority();
                get_engines();
            }
            else
                get_moments(false);
            moments_cycle_counter = (moments_cycle_counter + 1) % FullMomentFreq;
        }

        Vector3 get_sas_authority()
        {
            Vector3 res = Vector3.zero;
            foreach (var rw in vessel.FindPartModulesImplementing<ModuleReactionWheel>())
                if (rw.isEnabled && rw.wheelState == ModuleReactionWheel.WheelState.Active)
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
                if (pm.part.rb != null)
                {
                    result += pm.part.rb.worldCenterOfMass * pm.part.rb.mass;
                    mass += pm.part.rb.mass;
                }
                else
                {
                    mass += pm.part.mass + pm.part.GetResourceMass();
                    result += (pm.part.partTransform.position + pm.part.partTransform.rotation * pm.part.CoMOffset) *
                        (pm.part.mass + pm.part.GetResourceMass());
                }
            }
            if (mass > 0.0f)
                result /= mass;
            return result;
        }

        // Rotations of the currently controlling part of the vessel
        Quaternion world_to_cntrl_part;
        Quaternion cntrl_part_to_world;

        void get_moments(bool all_parts)
        {
            cntrl_part_to_world = vessel.transform.rotation;
            world_to_cntrl_part = cntrl_part_to_world.Inverse();            // from world to root part rotation
            CoM = vessel.findWorldCenterOfMass();                       // vessel.CoM unfortunately lags by one physics frame
            Vector3 world_v = vessel.rootPart.rb.velocity;
            Vector3 cur_CoM;
            if (all_parts)
            {
                MOI = Vector3.zero;
                AM = Vector3.zero;
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
            for (int pi = 0; pi < indexing; pi++)
            {
                Part part = all_parts ? vessel.parts[pi] : massive_parts[pi].part;
                if (part.physicalSignificance == Part.PhysicalSignificance.NONE)
                    continue;
                if (!part.isAttached || part.State == PartStates.DEAD)
                {
                    moments_cycle_counter = 0;      // iterating over old part list
                    continue;
                }
                Quaternion part_to_cntrl = part.partTransform.rotation * world_to_cntrl_part;   // from part to root part rotation
                Vector3 moi = Vector3.zero;
                Vector3 am = Vector3.zero;
                float mass = 0.0f;
                if (part.rb != null)
                {
                    mass = part.rb.mass;
                    Vector3 world_pv = part.rb.worldCenterOfMass - cur_CoM;
                    Vector3 pv = world_to_cntrl_part * world_pv;
                    Vector3 impulse = mass * (world_to_cntrl_part * (part.rb.velocity - world_v));
                    // from part.rb principal frame to root part rotation
                    Quaternion principal_to_root = part.rb.inertiaTensorRotation * part_to_cntrl;
                    // MOI of part as offsetted material point
                    moi += mass * new Vector3(pv.y * pv.y + pv.z * pv.z, pv.x * pv.x + pv.z * pv.z, pv.x * pv.x + pv.y * pv.y);
                    // MOI of part as rigid body
                    Vector3 rotated_moi = get_rotated_moi(part.rb.inertiaTensor, principal_to_root);
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
                    Vector3 impulse = mass * (world_to_cntrl_part * (part.vel - world_v));
                    // MOI of part as offsetted material point
                    moi += mass * new Vector3(pv.y * pv.y + pv.z * pv.z, pv.x * pv.x + pv.z * pv.z, pv.x * pv.x + pv.y * pv.y);
                    // angular moment of part as offsetted material point
                    am += Vector3.Cross(pv, impulse);
                }
                if (all_parts)
                {
                    massive_parts.Add(new PartMass(part, mass));
                    MOI += moi;
                    AM -= am;
                    sum_mass += mass;
                }
                else
                {
                    partial_MOI += moi;
                    partial_AM -= am;           // minus because fucking left handed unity
                }
            }
            if (all_parts)
            {
                massive_parts.Sort(PartMass.Comparison);
                if (massive_parts.Count > PartsMax)
                    massive_parts.RemoveRange(PartsMax, massive_parts.Count - PartsMax);
                angular_vel = Common.divideVector(AM, MOI);
            }
            else
                angular_vel = Common.divideVector(partial_AM, partial_MOI);
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
            // update surface velocity
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

			Vector3 projected_vel = up_srf_v + fwd_srf_v;
			if (projected_vel.sqrMagnitude > 1.0f)
			{
				float aoa_p = (float)Math.Asin(Common.Clampf(Vector3.Dot(vessel.ReferenceTransform.forward.normalized, projected_vel.normalized), 1.0f));
				if (Vector3.Dot(projected_vel, vessel.ReferenceTransform.up) < 0.0)
					aoa_p = (float)Math.PI - aoa_p;
				aoa_buf[PITCH].Put(aoa_p);
			}
			else
				aoa_buf[PITCH].Put(0.0f);
			
			projected_vel = up_srf_v + right_srf_v;
			if (projected_vel.sqrMagnitude > 1.0f)
			{
				float aoa_y = (float)Math.Asin(Common.Clampf(Vector3.Dot(vessel.ReferenceTransform.right.normalized, projected_vel.normalized), 1.0f));
				if (Vector3.Dot(projected_vel, vessel.ReferenceTransform.up) < 0.0)
					aoa_y = (float)Math.PI - aoa_y;
				aoa_buf[YAW].Put(aoa_y);
			}
			else
				aoa_buf[YAW].Put(0.0f);

			projected_vel = right_srf_v + fwd_srf_v;
			if (projected_vel.sqrMagnitude > 1.0f)
			{
				float aoa_r = (float)Math.Asin(Common.Clampf(Vector3.Dot(vessel.ReferenceTransform.forward.normalized, projected_vel.normalized), 1.0f));
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
                if (csurf_buf[i].Size >= 1)
                    csurf_buf[i].Put(far_exponential_blend(csurf_buf[i].getLast(), raw_input));
                else
                    csurf_buf[i].Put(raw_input);
            }
		}

        public static float far_exponential_blend(float prev, float desire)
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



        #region EnginesUpdate

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

        [AutoGuiAttr("engines torque", false, "G5")]
        public Vector3 engines_torque;

        //[AutoGuiAttr("engines thrust", false, "G5")]
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

        [AutoGuiAttr("engines_torque_k0", false, "G6")]
        Vector3 engines_torque_k0;

        [AutoGuiAttr("engines_torque_k1", false, "G6")]
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

        #endregion



        #region DynamicsUpdate

        [AutoGuiAttr("Lift acc", false, "G6")]
        public double lift_acc = 0.0;

        //[AutoGuiAttr("Slide acc", false, "G8")]
        public double slide_acc = 0.0;

        Vector3d vess2planet;
        double g_acc;

        [AutoGuiAttr("pitch_gravity_acc", false, "G6")]
        public double pitch_gravity_acc;

        [AutoGuiAttr("pitch_engine_acc", false, "G6")]
        public double pitch_engine_acc;

        //[AutoGuiAttr("yaw_gravity_acc", false, "G8")]
        public double yaw_gravity_acc;

        //[AutoGuiAttr("yaw_engine_acc", false, "G8")]
        public double yaw_engine_acc;

        void update_dynamics()
        {
            dyn_pressure = vessel.atmDensity * vessel.srfSpeed * vessel.srfSpeed;

            vess2planet = vessel.mainBody.position - vessel.transform.position;
            g_acc = vessel.mainBody.gMagnitudeAtCenter / vess2planet.sqrMagnitude;
            Vector3d tangent_axis = Vector3.Cross(vessel.srf_velocity, vessel.ReferenceTransform.right).normalized;
            double pitch_g_projection = Vector3d.Dot(tangent_axis, vess2planet.normalized);
            double pitch_total_acc = Vector3d.Dot(vessel.acceleration, tangent_axis);
            pitch_gravity_acc = g_acc * pitch_g_projection;
            pitch_engine_acc = Vector3d.Dot(cntrl_part_to_world * engines_thrust / sum_mass, tangent_axis);
            lift_acc = pitch_total_acc - pitch_gravity_acc - pitch_engine_acc;
            double yaw_g_projection = Vector3d.Dot(vessel.transform.right, vess2planet.normalized);
            double yaw_total_acc = Vector3d.Dot(vessel.acceleration, vessel.transform.right);
            yaw_gravity_acc = g_acc * yaw_g_projection;
            yaw_engine_acc = Vector3d.Dot(cntrl_part_to_world * engines_thrust / sum_mass, vessel.transform.right);
            slide_acc = yaw_total_acc - yaw_gravity_acc - yaw_engine_acc;
        }

        #endregion



        #region ModelIdentification

        //public Vector3d model_acc = Vector3d.zero;

        LinApprox pitch_torque_model = new LinApprox(2);
        LinApprox roll_torque_model = new LinApprox(4);
        LinApprox yaw_torque_model = new LinApprox(2);
        LinApprox pitch_lift_model = new LinApprox(1);
        LinApprox yaw_lift_model = new LinApprox(1);

        const int IMM_BUF_SIZE = 10;

        OnlineLinTrainer pitch_trainer, roll_trainer, yaw_trainer;
        OnlineLinTrainer pitch_lift_trainer, yaw_lift_trainer;
        OnlineLinTrainer[] trainers = new OnlineLinTrainer[3];

        void initialize_ann_tainers()
        {
            pitch_trainer = new OnlineLinTrainer(pitch_torque_model, IMM_BUF_SIZE, new int[] { 11, 11 },
                new double[] { -0.1, -0.1 }, new double[] { 0.1, 0.1 }, pitch_input_method, pitch_output_method);
            pitch_trainer.base_gen_weight = 5.0f;
            pitch_trainer.max_value_decay = 0.0005f;
            pitch_trainer.gen_limits_decay = 0.0005f;
            pitch_trainer.linear_time_decay = 0.008f;
            pitch_trainer.nonlin_time_decay = 0.05f;
            pitch_trainer.min_gen_weight = 0.02f;
            pitch_trainer.linear_err_criteria = 0.2f;
            trainers[0] = pitch_trainer;

            roll_trainer = new OnlineLinTrainer(roll_torque_model, IMM_BUF_SIZE, new int[] { 5, 5, 5, 5 },
                new double[] { -0.1, -0.1, -0.1, -0.05 }, new double[] { 0.1, 0.1, 0.1, 0.05 }, roll_input_method, roll_output_method);
            trainers[1] = roll_trainer;

            yaw_trainer = new OnlineLinTrainer(yaw_torque_model, IMM_BUF_SIZE, new int[] { 11, 11 },
                new double[] { -0.1, -0.2 }, new double[] { 0.1, 0.2 }, yaw_input_method, yaw_output_method);
            trainers[2] = yaw_trainer;

            pitch_lift_trainer = new OnlineLinTrainer(pitch_lift_model, IMM_BUF_SIZE, new int[] { 11 },
                new double[] { -0.1 }, new double[] { 0.1 }, pitch_lift_input_method, pitch_lift_output_method);
            pitch_lift_trainer.base_gen_weight = 1.0f;
            pitch_lift_trainer.max_value_decay = 0.0002f;
            pitch_lift_trainer.gen_limits_decay = 0.0005f;
            pitch_lift_trainer.linear_time_decay = 0.004f;
            pitch_lift_trainer.nonlin_time_decay = 0.05f;
            pitch_lift_trainer.min_gen_weight = 0.02f;
            pitch_lift_trainer.linear_err_criteria = 0.2f;

            yaw_lift_trainer = new OnlineLinTrainer(yaw_lift_model, IMM_BUF_SIZE, new int[] { 11 },
                new double[] { -0.1 }, new double[] { 0.1 }, yaw_lift_input_method, yaw_lift_output_method);
            yaw_lift_trainer.base_gen_weight = 1.0f;
            yaw_lift_trainer.max_value_decay = 0.0002f;
            yaw_lift_trainer.gen_limits_decay = 0.0005f;
            yaw_lift_trainer.linear_time_decay = 0.003f;
            yaw_lift_trainer.nonlin_time_decay = 0.05f;
            yaw_lift_trainer.min_gen_weight = 0.05f;
            yaw_lift_trainer.linear_err_criteria = 0.5f;
        }

        /// <summary>
        /// Current dynamic pressure = density * air_speed^2
        /// </summary>
        public double dyn_pressure = 1.0;

        // Trainer input methods
        void pitch_input_method(Vector v)
        {
            v[0] = aoa_buf[PITCH].getFromTail(1);   // we need AoA from previous physics frame
            v[1] = csurf_buf[PITCH].getLast();      // same for control surface position (note index diffirence)
        }

        double pitch_output_method()
        {
            return (angular_acc_buf[PITCH].getLast() - 
                (reaction_torque[PITCH] * input_buf[PITCH].getLast() + engines_torque[PITCH]) / MOI[PITCH]) / dyn_pressure * 1e4;
        }

        void roll_input_method(Vector v)
        {
            v[0] = aoa_buf[YAW].getFromTail(1);
            v[1] = angular_v_buf[ROLL].getFromTail(1);
            v[2] = csurf_buf[YAW].getLast();
            v[3] = csurf_buf[ROLL].getLast();
        }

        double roll_output_method()
        {
            return (angular_acc_buf[ROLL].getLast() -
                (reaction_torque[ROLL] * input_buf[ROLL].getLast() + engines_torque[ROLL]) / MOI[ROLL]) / dyn_pressure * 1e4;
        }

        void yaw_input_method(Vector v)
        {
            v[0] = aoa_buf[YAW].getFromTail(1);
            v[1] = csurf_buf[YAW].getLast();            
        }

        double yaw_output_method()
        {
            return (angular_acc_buf[YAW].getLast() -
                (reaction_torque[YAW] * input_buf[YAW].getLast() + engines_torque[YAW]) / MOI[YAW]) / dyn_pressure * 1e4;
        }

        void pitch_lift_input_method(Vector v)
        {
            v[0] = aoa_buf[PITCH].getFromTail(1);
        }

        double pitch_lift_output_method()
        {
            return lift_acc / dyn_pressure * 1e3 * sum_mass;        // we approximate lift force, not acceleration
        }

        void yaw_lift_input_method(Vector v)
        {
            v[0] = aoa_buf[YAW].getFromTail(1);
        }

        double yaw_lift_output_method()
        {
            return slide_acc / dyn_pressure * 1e3 * sum_mass;
        }

        // Training inputs updating
        void update_training_inputs()
        {
            int dt = (int)Math.Round(Time.fixedDeltaTime * 100.0f);
            if (!vessel.LandedOrSplashed && dyn_pressure >= 10.0)
            {
                pitch_trainer.UpdateState(dt);
                roll_trainer.UpdateState(dt);
                yaw_trainer.UpdateState(dt);
                pitch_lift_trainer.UpdateState(dt);
                yaw_lift_trainer.UpdateState(dt);
            }
        }

        // Training methods
        //[AutoGuiAttr("pitch_cpu", false)]
        int pitch_cpu = 0;

        //[AutoGuiAttr("roll_cpu", false)]
        int roll_cpu = 5;

        //[AutoGuiAttr("yaw_cpu", false)]
        int yaw_cpu = 0;

        //[AutoGuiAttr("pitch_lift_cpu", false)]
        int pitch_lift_cpu = 0;

        //[AutoGuiAttr("yaw_lift_cpu", false)]
        int yaw_lift_cpu = 5;

        //[AutoGuiAttr("CPU per update", true)]
        int CPU_TIME_FOR_FIXEDUPDATE = 5;

        const int TORQUE_DESCEND_COST = 10;
        const int LIFT_DESCEND_COST = 10;

        void update_cpu()
        {
            Interlocked.Add(ref pitch_cpu, CPU_TIME_FOR_FIXEDUPDATE);
            Interlocked.Add(ref roll_cpu, CPU_TIME_FOR_FIXEDUPDATE);
            Interlocked.Add(ref yaw_cpu, CPU_TIME_FOR_FIXEDUPDATE);
            Interlocked.Add(ref pitch_lift_cpu, CPU_TIME_FOR_FIXEDUPDATE);
            Interlocked.Add(ref yaw_lift_cpu, CPU_TIME_FOR_FIXEDUPDATE);
        }

        bool train_pitch_ann()
        {
            if (pitch_cpu >= TORQUE_DESCEND_COST)
            {
                Interlocked.Add(ref pitch_cpu, -TORQUE_DESCEND_COST);
                pitch_trainer.Train();
                return true;
            }
            return false;
        }

        bool train_roll_ann()
        {
            if (roll_cpu >= TORQUE_DESCEND_COST)
            {
                Interlocked.Add(ref roll_cpu, -TORQUE_DESCEND_COST);
                roll_trainer.Train();
                return true;
            }
            return false;
        }

        bool train_yaw_ann()
        {
            if (yaw_cpu >= TORQUE_DESCEND_COST)
            {
                Interlocked.Add(ref yaw_cpu, -TORQUE_DESCEND_COST);
                yaw_trainer.Train();
                return true;
            }
            return false;
        }

        bool train_pitch_lift()
        {
            if (pitch_lift_cpu >= LIFT_DESCEND_COST)
            {
                Interlocked.Add(ref pitch_lift_cpu, -LIFT_DESCEND_COST);
                pitch_lift_trainer.Train();
                return true;
            }
            return false;
        }

        bool train_yaw_lift()
        {
            if (yaw_lift_cpu >= LIFT_DESCEND_COST)
            {
                Interlocked.Add(ref yaw_lift_cpu, -LIFT_DESCEND_COST);
                yaw_lift_trainer.Train();
                return true;
            }
            return false;
        }

        // Model evaluation
        //Vector temp_v1 = new Vector(1);
        Vector temp_v2 = new Vector(2);
        //Vector temp_v3 = new Vector(3);
        Vector temp_v4 = new Vector(4);
        void update_model_acc()
        {
            //pitch_input_method(temp_v2);
            pitch_torque_model.update_from_training();
            //model_acc[PITCH] = pitch_torque_model.eval(temp_v2) / 1e4 * dyn_pressure;
            //roll_input_method(temp_v4);
            roll_torque_model.update_from_training();
            //model_acc[ROLL] = roll_torque_model.eval(temp_v4) / 1e4 * dyn_pressure;
            //yaw_input_method(temp_v2);
            yaw_torque_model.update_from_training();
            //model_acc[YAW] = yaw_torque_model.eval(temp_v2) / 1e4 * dyn_pressure;
            pitch_lift_model.update_from_training();
            yaw_lift_model.update_from_training();
        }

		#endregion



        #region RotationModels

        public readonly LinearSystemModel pitch_rot_model = new LinearSystemModel(3, 1);
        //public readonly LinearSystemModel roll_rot_model = new LinearSystemModel(3, 1);
        public readonly LinearSystemModel yaw_rot_model = new LinearSystemModel(3, 1);

        public struct RegressionCoeffs
        {
            public double k0;
            public double k1;
            public double k2;
            public double Cl0;
            public double Cl1;
            public double et0;
            public double et1;
        }

        public RegressionCoeffs pitch_coeffs, yaw_coeffs;

        void update_pitch_rot_model()
        {
            // Fill coeff structs
            pitch_coeffs.Cl0 = pitch_lift_model.pars[0] / 1e3 * dyn_pressure / sum_mass;
            pitch_coeffs.Cl1 = pitch_lift_model.pars[1] / 1e3 * dyn_pressure / sum_mass;
            pitch_coeffs.et0 = engines_torque_k0[PITCH] / MOI[PITCH];
            pitch_coeffs.et1 = engines_torque_k1[PITCH] / MOI[PITCH];
            pitch_coeffs.k0 = pitch_torque_model.pars[0] / 1e4 * dyn_pressure;
            pitch_coeffs.k1 = pitch_torque_model.pars[1] / 1e4 * dyn_pressure;
            pitch_coeffs.k2 = pitch_torque_model.pars[2] / 1e4 * dyn_pressure;

            // Fill linear model
            Matrix A = pitch_rot_model.A;            
            if (dyn_pressure >= 10.0)
                A[0, 0] = -(pitch_coeffs.Cl1 + engines_thrust[ROLL] / sum_mass) / vessel.srfSpeed;
            else
                A[0, 0] = 0.0;
            A[0, 1] = 1.0;
            A[1, 0] = pitch_coeffs.k1;
            A[1, 2] = pitch_coeffs.k2 * (1.0 - 4.0 * TimeWarp.fixedDeltaTime);
            A[2, 2] = -4.0;
            Matrix B = pitch_rot_model.B;
            B[1, 0] = reaction_torque[PITCH] / MOI[PITCH] + pitch_coeffs.k2 * 4.0 * TimeWarp.fixedDeltaTime + pitch_coeffs.et1;
            B[2, 0] = 4.0;
            Matrix C = pitch_rot_model.C;
            if (dyn_pressure >= 10.0)
            {
                C[0, 0] = -(pitch_gravity_acc + pitch_coeffs.Cl0 - engines_thrust[YAW] / sum_mass) / vessel.srfSpeed;
                C[1, 0] = pitch_coeffs.k0 + pitch_coeffs.et0;
            }
            else
            {
                C[0, 0] = 0.0;
                C[1, 0] = pitch_coeffs.et0;
            }
            //Debug.Log("A =\r\n" + A.ToString() + "B =\r\n" + B.ToString() + "C =\r\n" + C.ToString());
        }

        void update_yaw_rot_model()
        {
            // Fill coeff structs
            yaw_coeffs.Cl0 = yaw_lift_model.pars[0] / 1e3 * dyn_pressure / sum_mass;
            yaw_coeffs.Cl1 = yaw_lift_model.pars[1] / 1e3 * dyn_pressure / sum_mass;
            yaw_coeffs.k0 = yaw_torque_model.pars[0] / 1e4 * dyn_pressure;
            yaw_coeffs.k1 = yaw_torque_model.pars[1] / 1e4 * dyn_pressure;
            yaw_coeffs.k2 = yaw_torque_model.pars[2] / 1e4 * dyn_pressure;

            // Fill linear model
            Matrix A = yaw_rot_model.A;
            if (dyn_pressure >= 10.0)
                A[0, 0] = -yaw_coeffs.Cl1 / vessel.srfSpeed;
            else
                A[0, 0] = 0.0;
            A[0, 1] = 1.0;
            A[1, 0] = yaw_coeffs.k1;
            A[1, 2] = yaw_coeffs.k2 * (1.0 - 4.0 * TimeWarp.fixedDeltaTime);
            A[2, 2] = -4.0;
            Matrix B = yaw_rot_model.B;
            B[1, 0] = reaction_torque[YAW] / MOI[YAW] + yaw_coeffs.k2 * 4.0 * TimeWarp.fixedDeltaTime;
            B[2, 0] = 4.0;
            Matrix C = yaw_rot_model.C;
            if (dyn_pressure >= 10.0)
                C[0, 0] = -(yaw_gravity_acc + yaw_coeffs.Cl0 + yaw_engine_acc) / vessel.srfSpeed;
            else
                C[0, 0] = 0.0;
            C[1, 0] = yaw_coeffs.k0;
        }

        #endregion



        #region Serialization

        //protected override void OnDeserialize(ConfigNode node, Type attribute_type)
        //{
        //    if (attribute_type == typeof(VesselSerializable))
        //    {
        //        SimpleAnn pann = SimpleAnn.DeserializeFromNode(node, "pitch_ann");
        //        if (pann != null)
        //            pitch_ann = pann;
        //        SimpleAnn rann = SimpleAnn.DeserializeFromNode(node, "roll_ann");
        //        if (rann != null)
        //            roll_ann = rann;
        //        SimpleAnn yann = SimpleAnn.DeserializeFromNode(node, "yaw_ann");
        //        if (yann != null)
        //            yaw_ann = yann;
        //    }
        //}

        //protected override void OnSerialize(ConfigNode node, Type attribute_type)
        //{
        //    if (attribute_type == typeof(VesselSerializable))
        //    {
        //        pitch_ann.SerializeToNode(node, "pitch_ann");
        //        roll_ann.SerializeToNode(node, "roll_ann");
        //        yaw_ann.SerializeToNode(node, "yaw_ann");
        //    }
        //}

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
                //GUILayout.Label("ang vel part = " + vessel.angularVelocity[i].ToString("G8"), GUIStyles.labelStyleLeft);
                GUILayout.Label("ang acc = " + angular_acc_buf[i].getLast().ToString("G8"), GUIStyles.labelStyleLeft);
                GUILayout.Label("AoA = " + (aoa_buf[i].getLast() * rad2degree).ToString("G8"), GUIStyles.labelStyleLeft);
                //GUILayout.Label("MOI = " + MOI[i].ToString("G8"), GUIStyles.labelStyleLeft);
                //GUILayout.Label("AngMoment = " + AM[i].ToString("G8"), GUIStyles.labelStyleLeft);
                if (i == 0)
                    AutoGUI.AutoDrawObject(trainers[i]);
			}
            GUILayout.Space(5.0f);
            GUILayout.Label("Pitch lift trainer", GUIStyles.labelStyleLeft);
            AutoGUI.AutoDrawObject(pitch_lift_trainer);
            GUILayout.Space(5.0f);
            //GUILayout.Label("Yaw lift trainer", GUIStyles.labelStyleLeft);
            //AutoGUI.AutoDrawObject(yaw_lift_trainer);
            //GUILayout.Space(5.0f);
            AutoGUI.AutoDrawObject(this);
			GUILayout.EndVertical();
			GUI.DragWindow();
        }

        #endregion
    }
}
