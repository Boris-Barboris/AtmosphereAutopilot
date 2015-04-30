using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace AtmosphereAutopilot
{
    class VesselIDComparator : IEqualityComparer<Vessel>
    {
        public int GetHashCode(Vessel foo) { return foo.id.GetHashCode(); }
        public bool Equals(Vessel foo1, Vessel foo2) { return foo1.id == foo2.id; }
    }

    public static class Common
    {
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
