using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;

namespace AtmosphereAutopilot
{
    class VesselIDComparator : IEqualityComparer<Vessel>
    {
        public int GetHashCode(Vessel foo) { return foo.id.GetHashCode(); }
        public bool Equals(Vessel foo1, Vessel foo2) { return foo1.id == foo2.id; }
    }

    public static class Common
    {
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
        public static Matrix4x4 rotationMatrix(Quaternion q)
        {
            Matrix4x4 mat = Matrix4x4.zero;
            mat[3, 3] = 1.0f;
            q = normalizeQuaternion(q);
            mat[0, 0] = 1.0f - 2.0f * q.y * q.y - 2.0f * q.z * q.z;
            mat[1, 0] = 2.0f * q.x * q.y + 2.0f * q.z * q.w;
            mat[2, 0] = 2.0f * q.x * q.z - 2.0f * q.y * q.w;
            mat[0, 1] = 2.0f * q.x * q.y - 2.0f * q.z * q.w;
            mat[1, 1] = 1.0f - 2.0f * q.x * q.x - 2.0f * q.z * q.z;
            mat[2, 1] = 2.0f * q.y * q.z + 2.0f * q.x * q.w;
            mat[0, 2] = 2.0f * q.x * q.z + 2.0f * q.y * q.w;
            mat[1, 2] = 2.0f * q.y * q.z - 2.0f * q.x * q.w;
            mat[2, 2] = 1.0f - 2.0f * q.x * q.x - 2.0f * q.y * q.y;
            return mat;
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
    }
}
