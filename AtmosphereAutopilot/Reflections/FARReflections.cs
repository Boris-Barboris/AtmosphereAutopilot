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
using System.Reflection;
using UnityEngine;

namespace AtmosphereAutopilot
{
    public class FARReflections
    {
        public Assembly farAssembly;
        public bool isFarFound = false;

        public Type FARControllableSurfaceType;

        public delegate double ActiveVesselIASDelegateType();
        public ActiveVesselIASDelegateType ActiveVesselIASDelegate;

        public delegate double RayleighPitotTubeStagPressureDelegateType(double M);
        public RayleighPitotTubeStagPressureDelegateType RayleighPitotTubeStagPressureDelegate;

        public FARReflections()
        {
            farAssembly = ReflectionUtils.GetAssembly("FerramAerospaceResearch");

            if (farAssembly != null)
            {
                Debug.Log("[AtmosphereAutopilot]: FAR aerodynamics detected");
                isFarFound = true;

                // Get FAR control surface type
                FARControllableSurfaceType = ReflectionUtils.GetType(farAssembly, "FARControllableSurface");
                if (FARControllableSurfaceType == null)
                {
                    throw new Exception("AtmosphereAutopilot could not bind to FAR FARControllableSurface class");
                }

                // Load ActiveVesselIAS method from FARAPI
                var activeVesselIASMethodInfo = ReflectionUtils.GetMethodInfo(
                    farAssembly,
                    "FerramAerospaceResearch.FARAPI",
                    "ActiveVesselIAS",
                    BindingFlags.Public | BindingFlags.Static);

                if (activeVesselIASMethodInfo == null)
                {
                    Debug.LogWarning("[AtmosphereAutopilot]: FAR ActiveVesselIAS() method reflection failed, disabling FAR IAS support");
                    // don't load other methods if ActiveVesselIAS fails
                    return;
                } 
                else
                {
                    ActiveVesselIASDelegate = (ActiveVesselIASDelegateType)
                        Delegate.CreateDelegate(
                            typeof(ActiveVesselIASDelegateType),
                            activeVesselIASMethodInfo);
                }
                
                // Load undocumented RayleighPitotTubeStagPressure method from FAR
                var rayleighPitotTubeStagPressureMethodInfo = ReflectionUtils.GetMethodInfo(
                    farAssembly,
                    "FerramAerospaceResearch.FARAeroUtil",
                    "RayleighPitotTubeStagPressure",
                    BindingFlags.Public | BindingFlags.Static,
                    new Type[] { typeof(double) });

                if (rayleighPitotTubeStagPressureMethodInfo == null)
                {
                    Debug.LogWarning("[AtmosphereAutopilot]: FAR RayleighPitotTubeStagPressure() method reflection failed, reverting to EAS speed conversions");
                }
                else
                {
                    RayleighPitotTubeStagPressureDelegate = (RayleighPitotTubeStagPressureDelegateType)
                        Delegate.CreateDelegate(
                            typeof(RayleighPitotTubeStagPressureDelegateType),
                            rayleighPitotTubeStagPressureMethodInfo);
                }
            }
        }
    }
}
