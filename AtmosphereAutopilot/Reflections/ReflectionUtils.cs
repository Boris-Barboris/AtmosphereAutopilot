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
using System.Linq;
using System.Reflection;

namespace AtmosphereAutopilot
{
    public static class ReflectionUtils
    {
        public static Assembly GetAssembly(string module)
        {
            foreach (var a in AppDomain.CurrentDomain.GetAssemblies())
            {
                if (a.GetName().Name.Equals(module))
                {
                    return a;
                }
            }
            return null;
        }
        public static MethodInfo GetMethodInfo(Assembly assembly, string className, string methodName, BindingFlags flags, Type[] args = null)
        {
            if (assembly == null)
                return null;

            Type type = assembly.GetType(className, false);

            if (type == null)
                return null;

            if (args == null)
                args = Type.EmptyTypes;

            return type.GetMethod(methodName, flags, null, args, null);
        }

        public static Type GetType(Assembly assembly, string typeName)
        {
            return assembly.GetTypes().First(t => t.Name.Equals(typeName));
        }
    }
}
