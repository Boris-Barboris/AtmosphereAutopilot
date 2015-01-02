using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace AtmosphereAutopilot
{
    class VesselIDComparator : IEqualityComparer<Vessel>
    {
        public int GetHashCode(Vessel foo) { return foo.id.GetHashCode().GetHashCode(); }
        public bool Equals(Vessel foo1, Vessel foo2) { return foo1.id == foo2.id; }
    }

    static class Common
    {
        public static double Clamp(double val, double under, double upper)
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
    }
}
