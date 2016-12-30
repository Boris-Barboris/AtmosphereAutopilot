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
using System.Runtime.CompilerServices;

namespace AtmosphereAutopilot
{
    class VesselOldComparator : IEqualityComparer<Vessel>
    {
        public int GetHashCode(Vessel foo) { return RuntimeHelpers.GetHashCode(foo); }
        public bool Equals(Vessel foo1, Vessel foo2) { return RuntimeHelpers.ReferenceEquals(foo1, foo2); }
    }

    class VesselIDComparator : IEqualityComparer<Vessel>
    {
        public int GetHashCode(Vessel foo) { return foo.id.GetHashCode(); }
        public bool Equals(Vessel foo1, Vessel foo2) { return foo1.id == foo2.id; }
    }

    public static class Common
    {
        /// <summary>
        /// Reallocate array if needed
        /// </summary>
        /// <param name="capacity">Required array capacity</param>
        /// <param name="storage">Storage to try to reuse</param>
        public static void Realloc<T>(ref T[] storage, int capacity)
        {
            if (storage == null || capacity > storage.Length)
                storage = new T[capacity];
        }

        public static Quaternion normalizeQuaternion(Quaternion quat)
        {
            float n = (float)Math.Sqrt(quat.x * quat.x + quat.y * quat.y + quat.z * quat.z + quat.w * quat.w);
            quat.x /= n;
            quat.y /= n;
            quat.z /= n;
            quat.w /= n;
            return quat;
        }

        /// <summary>
        /// http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToMatrix/index.htm
        /// </summary>
        /// <param name="q">Rotation</param>
        /// <returns>Transformation matrix representing rotation</returns>
        public static void rotationMatrix(Quaternion q, Matrix mat)
        {
            q = normalizeQuaternion(q);
            mat[0, 0] = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
            mat[1, 0] = 2.0f * (q.x * q.y + q.z * q.w);
            mat[2, 0] = 2.0f * (q.x * q.z - q.y * q.w);
            mat[0, 1] = 2.0f * (q.x * q.y - q.z * q.w);
            mat[1, 1] = 1.0f - 2.0f * (q.x * q.x + q.z * q.z);
            mat[2, 1] = 2.0f * (q.y * q.z + q.x * q.w);
            mat[0, 2] = 2.0f * (q.x * q.z + q.y * q.w);
            mat[1, 2] = 2.0f * (q.y * q.z - q.x * q.w);
            mat[2, 2] = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
        }

        public static double simple_filter(double new_value, double old_value, double k)
        {
            return (old_value * k + new_value) / (k + 1.0);
        }

        public static double lerp(double a, double b, double k)
        {
            k = Common.Clamp(k, 1.0);
            return a * (1.0 - k) + b * k;
        }

        public static Vector3 divideVector(Vector3 lhs, Vector3 rhs)
        {
            Vector3 result = new Vector3(lhs.x / rhs.x, lhs.y / rhs.y, lhs.z / rhs.z);
            return result;
        }

        public static double Meansqr(this ICollection<double> col)
        {
            double sqr_sum = 0.0;
            foreach (double i in col)
                sqr_sum += i * i;
            return sqr_sum / col.Count;
        }

        public static double Meansqr(this ICollection<double> col, int count, ICollection<double> weights = null)
        {
            double sqr_sum = 0.0;
            int r_count = 0;
            var en = col.GetEnumerator();
            IEnumerator<double> wen = null;
            if (weights != null)
                wen = weights.GetEnumerator();
            while (r_count < count)
            {
                if (en.MoveNext())
                {
                    r_count++;
                    double val = en.Current;
                    if (wen != null && wen.MoveNext())
                        val *= wen.Current;             
                    sqr_sum += val * val;
                }
                else
                    break;
            }
            return sqr_sum / (double)r_count;
        }

        public static int Clamp(int val, int under, int upper)
        {
            if (under > val)
                return under;
            if (upper < val)
                return upper;
            return val;
        }

        public static double Clamp(double val, double under, double upper)
        {
            if (under > val)
                return under;
            if (upper < val)
                return upper;
            return val;
        }

        public static float Clampf(float val, float under, float upper)
        {
            if (under > val)
                return under;
            if (upper < val)
                return upper;
            return val;
        }

        public static double Clamp(double val, double limit)
        {
            limit = Math.Abs(limit);
            return Clamp(val, -limit, limit);
        }

        public static float Clampf(float val, float limit)
        {
            limit = Math.Abs(limit);
            return Clampf(val, -limit, limit);
        }

        public static double derivative1_short(double y0, double y1, double dt)    // first derivative
        {
            return (y1 - y0) / dt;
        }

        public static double derivative1_middle(double y0, double y2, double dt)    // first derivative
        {
            return (y2 - y0) / dt * 0.5;
        }

        public static double derivative1(double y0, double y1, double y2, double dt)    // first derivative
        {
            return (y0 - 4 * y1 + 3 * y2) / dt * 0.5;
        }

        public static double derivative2(double y0, double y1, double y2, double dt)    // second derivative
        {
            return (y0 - 2 * y1 + y2) / dt / dt;
        }

        public static double derivative2_long(double y0, double y1, double y2, double y3, double dt)
        {
            return (-y0 + 4 * y1 - 5 * y2 + 2 * y3) / dt / dt;
        }

        public static double extrapolate(double y0, double dy1, double dy2, double dt)
        {
            return y0 + dy1 * dt + 0.5 * dy2 * dt * dt;
        }

        public static string ToString(this Vector3d v, string format)
        {
            return '(' + v.x.ToString(format) + ", " + v.y.ToString(format) + ", " + v.z.ToString(format) + ')';
        }
    }


    public static class QuaternionExtensions
    {
        public static Quaternion Normalize(this Quaternion q)
        {
            Quaternion result;
            float sq = q.x * q.x;
            sq += q.y * q.y;
            sq += q.z * q.z;
            sq += q.w * q.w;
            float inv = (float)(1.0 / Math.Sqrt(sq));
            result.x = q.x * inv;
            result.y = q.y * inv;
            result.z = q.z * inv;
            result.w = q.w * inv;
            return result;
        }
    }

}
