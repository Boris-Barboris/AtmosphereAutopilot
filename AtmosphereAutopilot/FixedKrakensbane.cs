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
using System.Reflection;

namespace FixedKrakensbane
{
    [KSPAddon(KSPAddon.Startup.Flight, false)]
    public class FixedKrakensbane : MonoBehaviour
    {
        public static FixedKrakensbane Instance { get; private set; }

        public float MaxVelocity
        {
            get
            {
                return _maxv;
            }
            private set
            {
                _maxv = value;
                if (krak_instance != null)
                    krak_instance.MaxV = _maxv;
            }
        }
        float _maxv = 750.0f;

        public Vector3d FrameVelocity 
        {
            get
            {
                return _fv;
            }
            private set
            {
                _fv = value;
                if (krak_instance != null)
                    krak_instance.FrameVel = _fv;
            }
        }

        public Vector3d LastCorrection
        {
            get
            {
                return _lc;
            }
            private set
            {
                _lc = value;
                if (krak_instance != null)
                    krak_instance.lastCorrection = _lc;
            }
        }

        Vector3d _fv;
        Vector3d _lc;

        void Awake()
        {
            if (Instance != null && Instance != this)
                UnityEngine.Object.Destroy(Instance);
            Debug.Log("FixedKrakensbane.Awake()");
            Instance = this;
        }

        static Krakensbane krak_instance;

        void Start()
        {
            Debug.Log("FixedKrakensbane.Start()");
            // Initialize frames to zero
            FrameVelocity = Vector3d.zero;
            // Krakensbane handling
            if (krak_instance == null)
            {
                // Find and destroy Squad's Krakensbane
                FieldInfo kraken_field = (from field in typeof(Krakensbane).GetFields(BindingFlags.Static | BindingFlags.NonPublic)
                                          where field.FieldType.Equals(typeof(Krakensbane))
                                          select field).First();
                if (kraken_field != null)
                {
                    Debug.Log("Krakensbane static field is found");
                    krak_instance = kraken_field.GetValue(null) as Krakensbane;
                    if (krak_instance)
                    {
                        //UnityEngine.Object.Destroy(krak_instance);
                        krak_instance.enabled = false;
                        Debug.Log("Krakensbane is disabled");
                        krak_instance.MaxV = MaxVelocity;
                        krak_instance.FrameVel = FrameVelocity;
                    }
                }
            }
            else
                Debug.Log("Krakensbane was already off");
        }

        double max_proj(Vector3d v)
        {
            return Math.Max(Math.Abs(v.x), Math.Max(Math.Abs(v.y), Math.Abs(v.z)));
        }

        float max_proj(Vector3 v)
        {
            return Math.Max(Math.Abs(v.x), Math.Max(Math.Abs(v.y), Math.Abs(v.z)));
        }

        void FixedUpdate()
        {
            if (!FlightGlobals.ready)
                return;
            Vessel cur_ves = FlightGlobals.ActiveVessel;
            if (cur_ves == null)
                return;
            Rigidbody root_rb = cur_ves.rootPart.rigidbody;
            if (root_rb != null)
            {
                Vector3 ves_vel = root_rb.velocity;
                LastCorrection = Vector3d.zero;
                if (!cur_ves.packed)
                {
                    if (cur_ves.state != Vessel.State.DEAD)
                    {
                        // check if we are speeding
                        if (max_proj(ves_vel) > MaxVelocity)
                        {
                            // and we are
                            Vector3 shift_vel = ves_vel;
                            // Shoot event if needed
                            if (FrameVelocity.IsZero())
                                GameEvents.onKrakensbaneEngage.Fire(-shift_vel);
                            FrameVelocity += shift_vel;
                            LastCorrection = shift_vel;
                            // update velocities
                            offset_velocities(-shift_vel);
                        }
                        else
                        {
                            // check if we can return to original non-moving inertia frame
                            if (!FrameVelocity.IsZero() && (max_proj(ves_vel + FrameVelocity) < MaxVelocity * 0.75f))
                            {
                                Vector3 shift_vel = -FrameVelocity;
                                GameEvents.onKrakensbaneDisengage.Fire(FrameVelocity);
                                FrameVelocity = Vector3d.zero;
                                LastCorrection = shift_vel;
                                // update velocities
                                offset_velocities(-shift_vel);
                            }
                        }
                    }
                }
                else
                {
                    FrameVelocity = Vector3d.zero;
                }
                // Move all affected by Krakensbane bodys according to FrameVelocity
                if (!FrameVelocity.IsZero())
                    offset_positions(-FrameVelocity * Time.fixedDeltaTime);
            }
        }

        void offset_velocities(Vector3 vel_offset)
        {
            // update unpacked loaded vessels
            var vessels_to_change =
                from vessel in FlightGlobals.Vessels
                where (vessel != null) && vessel.loaded && !vessel.packed && (vessel.state != Vessel.State.DEAD)
                select vessel;
            foreach (var v in vessels_to_change)
                v.ChangeWorldVelocity(vel_offset);
            // update physical objects
            var obj_to_change =
                from obj in FlightGlobals.physicalObjects
                where (obj != null) && (obj.rigidbody != null)
                select obj.rigidbody;
            foreach (var rb in obj_to_change)
                rb.AddForce(vel_offset, ForceMode.VelocityChange);
        }

        void offset_positions(Vector3d offset)
        {
            for (int i = 0; i < FlightGlobals.Bodies.Count; i++)
            {
                FlightGlobals.Bodies[i].position += offset;
            }
            for (int j = 0; j < FlightGlobals.Vessels.Count; j++)
            {
                Vessel vessel = FlightGlobals.Vessels[j];
                if (vessel.state == Vessel.State.DEAD)
                    continue;
                else
                {
                    for (int k = 0; k < vessel.parts.Count; k++)
                    {
                        Part part = vessel.parts[k];
                        for (int l = 0; l < part.fxGroups.Count; l++)
                        {
                            FXGroup fXGroup = part.fxGroups[l];
                            if (!fXGroup.Active)
                                continue;
                            else
                                for (int m = 0; m < fXGroup.fxEmitters.Count; m++)
                                {
                                    ParticleEmitter particleEmitter = fXGroup.fxEmitters[m];
                                    if (!particleEmitter.useWorldSpace)
                                        continue;
                                    else
                                    {
                                        Particle[] particles = particleEmitter.particles;
                                        for (int n = 0; n < particles.Length; n++)
                                        {
                                            Particle p = particles[n];
                                            p.position += offset;
                                            particles[n] = p;
                                        }
                                        particleEmitter.particles = particles;
                                    }
                                }
                        }
                    }
                    if (!vessel.loaded || vessel.packed)
                        vessel.SetPosition(vessel.transform.position + offset);                 
                }
            }
            EffectBehaviour.OffsetParticles(-offset);
        }
    }
}
