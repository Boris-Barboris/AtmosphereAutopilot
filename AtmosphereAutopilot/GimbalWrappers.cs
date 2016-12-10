using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;
using System.Reflection;

namespace AtmosphereAutopilot
{
    public interface IGimbal
    {
        bool Active { get; set; }
        bool UseGimbalSpeed { get; set; }
        float GimbalSpeed { get; set; }
        void Modify(float gimbal_spd);
        void Restore();
        Quaternion neutralLocalRotation(int thrustTransformIndex);
    }

    public class StockGimbal : IGimbal
    {
        ModuleGimbal module;

        public StockGimbal(PartModule g)
        {
            module = (ModuleGimbal)g;
            original_spd = GimbalSpeed;
            original_use_spd = UseGimbalSpeed;
        }

        public bool Active
        {
            get { return !module.gimbalLock; }
            set { module.gimbalLock = !value; }
        }

        public bool UseGimbalSpeed
        {
            get { return module.useGimbalResponseSpeed; }
            set { module.useGimbalResponseSpeed = value; }
        }

        public float GimbalSpeed
        {
            get { return module.gimbalResponseSpeed; }
            set { module.gimbalResponseSpeed = value; }
        }

        float original_spd;
        bool original_use_spd;

        public void Modify(float gimbal_spd)
        {
            original_use_spd = UseGimbalSpeed;
            if (!float.IsPositiveInfinity(gimbal_spd))
                UseGimbalSpeed = true;
            original_spd = GimbalSpeed;
            GimbalSpeed = gimbal_spd;
        }

        public void Restore()
        {
            UseGimbalSpeed = original_use_spd;
            GimbalSpeed = original_spd;
        }

        public Quaternion neutralLocalRotation(int thrustTransformIndex)
        {
            // strange behaviour of sstu-s engines, need to check bounds
            int index = Math.Min(module.initRots.Count - 1, thrustTransformIndex);
            if (index < 0)
                return Quaternion.identity;
            return module.initRots[index];
        }
    }

    public class kmGimbal : IGimbal
    {
        public static Type gimbal_type;
        static FieldInfo f_enableGimbal, f_enableSmoothGimbal,
            f_useExponentGimbal, f_expResponseSpeed;

        internal static bool do_reflections()
        {
            try
            {
                Assembly km_asm = AppDomain.CurrentDomain.GetAssemblies().First(a => a.GetName().Name.Equals("km_Gimbal"));
                if (km_asm == null)
                    return false;
                Debug.Log("[AtmosphereAutopilot]: km_Gimbal assembly version " + km_asm.GetName().Version.ToString() + " found");
                gimbal_type = km_asm.GetType("km_Gimbal.KM_Gimbal_3");
                if (gimbal_type == null)
                    return false;
                f_enableGimbal = gimbal_type.GetField("enableGimbal", BindingFlags.Instance | BindingFlags.Public);
                f_enableSmoothGimbal = gimbal_type.GetField("enableSmoothGimbal", BindingFlags.Instance | BindingFlags.Public);
                f_useExponentGimbal = gimbal_type.GetField("useExponentGimbal", BindingFlags.Instance | BindingFlags.Public);
                f_expResponseSpeed = gimbal_type.GetField("expResponseSpeed", BindingFlags.Instance | BindingFlags.Public);
                if (f_enableGimbal == null || f_enableSmoothGimbal == null || f_useExponentGimbal == null || f_expResponseSpeed == null)
                    return false;
                else
                    Debug.Log("[AtmosphereAutopilot]: km_Gimbal is reflected without errors");
                return true;
            }
            catch
            {
                return false;
            }
        }

        PartModule km_gimbal;

        public kmGimbal(PartModule g)
        {
            km_gimbal = g;
            original_spd = GimbalSpeed;
            original_smooth = (bool)f_enableSmoothGimbal.GetValue(km_gimbal);
        }

        public bool Active
        {
            get { return (bool)f_enableGimbal.GetValue(km_gimbal); }
            set { f_enableGimbal.SetValue(km_gimbal, value); }
        }

        public bool UseGimbalSpeed
        {
            get { return true; }
            set {}
        }

        public float GimbalSpeed
        {
            get { return (float)f_expResponseSpeed.GetValue(km_gimbal); }
            set { f_expResponseSpeed.SetValue(km_gimbal, value); }
        }

        float original_spd;
        bool original_smooth;

        public void Modify(float gimbal_spd)
        {
            original_smooth = (bool)f_enableSmoothGimbal.GetValue(km_gimbal);
            if (!original_smooth)
                f_enableSmoothGimbal.SetValue(km_gimbal, true);
            original_spd = GimbalSpeed;
            GimbalSpeed = gimbal_spd;
            f_useExponentGimbal.SetValue(km_gimbal, true);
        }

        public void Restore()
        {
            f_enableSmoothGimbal.SetValue(km_gimbal, original_smooth);
            f_useExponentGimbal.SetValue(km_gimbal, false);
            GimbalSpeed = original_spd;
        }

        public Quaternion neutralLocalRotation(int thrustTransformIndex)
        {
            // dummy method, deprecated plugin anyways
            return Quaternion.identity;
        }
    }
}
