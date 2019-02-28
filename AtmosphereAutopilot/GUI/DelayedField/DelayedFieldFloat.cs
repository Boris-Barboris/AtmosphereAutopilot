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

//
// THIS FILE IS AUTO-GENERATED
//

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;

namespace AtmosphereAutopilot
{

    /// <summary>
    /// Struct for organizing delayed textfield input. Use DisplayLayout() to integrate into OnGUI code,
	/// use OnUpdate() to account for time.
    /// </summary>
    public struct DelayedFieldFloat
    {
        /// <summary>
        /// Underlying float value to represent
        /// </summary>
        float val;

        /// <summary>
        /// Time in seconds after input string was changed by user input
        /// </summary>
        float time;

        /// <summary>
        /// true when we're counting time
        /// </summary>
        bool changed;

        /// <summary>
        /// String that holds input value
        /// </summary>
        public string input_str;

        /// <summary>
        /// String that holds conversion formatting
        /// </summary>
        public string format_str;

	    /// <summary>
	    /// lat/lon may be entered in deg min secHemi format
	    /// e.g. 74 39 39W
	    /// Only permits entry via OnUpdate; not via the setter
	    /// </summary>
	    public enum CoordFormat { Decimal, NS, EW };
		public CoordFormat coord_format;

        /// <summary>
        /// 
        /// </summary>
        /// <param name="init_value">Initial value</param>
        /// <param name="format"></param>
        public DelayedFieldFloat(
			float init_value,
			string format,
			CoordFormat coord_fmt = CoordFormat.Decimal)
        {
            val = init_value;
            time = 0.0f;
            changed = false;
            input_str = val.ToString(format);
            format_str = format;
	        coord_format = coord_fmt;
        }

        public static implicit operator float(DelayedFieldFloat f)
        {
            return f.val;
        }

        public float Value
        {
            get { return val; }
            set
            {
                if (value != val)
                {
                    changed = false;
                    time = 0.0f;
                    input_str = value.ToString(format_str);
                    val = value;
                }
            }
        }

		public void DisplayLayout(GUIStyle style, params GUILayoutOption[] options)
        {
            string new_str = GUILayout.TextField(input_str, style, options);
            if (!input_str.Equals(new_str))
            {
                changed = true;
                time = 0.0f;
            }
            input_str = new_str;
        }

		public CoordFormat check_coord_format()
		{
			return coord_format;
		}

		public void OnUpdate()
        {
            if (changed)
                time += Time.deltaTime;
            if (time >= 2.0f)
            {
                time = 0.0f;
                changed = false;
				if (coord_format != CoordFormat.Decimal) {
					// attempt to decode dms format indicated by NO leading
					// magnitude sign and by a trailing N, S, E or W
					var trimmed = input_str.Trim();
					var len = trimmed.Length;
					if (len > 0 && trimmed[0] != '+' && trimmed[0] != '-')
					{
						var last = trimmed[len - 1];
						int sign = 0;
						if (coord_format == CoordFormat.NS)
						{
							if (last == 'S' || last == 's') sign = -1;
							else if (last == 'N' || last == 'n') sign = 1;
						}
						else
						{
							if (last == 'W' || last == 'w') sign = -1;
							else if (last == 'E' || last == 'e') sign = 1;
						}
						trimmed = trimmed.Substring(0, len - 1);
						var dms = trimmed.Split(' ');
						if (sign != 0)
						{
							int num;
							int.TryParse(dms[0], out num);
							val = sign * num;
							if (dms.Length > 1)
							{
								int.TryParse(dms[1], out num);
								val += sign * num / 60.0f;
								if (dms.Length > 2)
								{
									int.TryParse(dms[1], out num);
									val += sign * num / 3600.0f;
								}
							}
							input_str = val.ToString(format_str);	// feedback
							Debug.Log($"[AtmosphereAutopilot] coords {val:F4}");
							return;
						}
						if (dms.Length > 1) return;	// wait for NS/EW
					}
				}
                float.TryParse(input_str, out val);
			}
        }

		public override string ToString()
        {
            return val.ToString() + "_" + format_str;
        }

        public string ToString(string format)
        {
            return val.ToString(format) + "_" + format_str;
        }

        public static DelayedFieldFloat Parse(string str)
        {
            int delim_index = str.IndexOf('_');
            if (delim_index < 0)
			{
				return new DelayedFieldFloat(0.0f, "G4");
			}
            else
            {
                string val_str = str.Substring(0, delim_index);
                string format_str = str.Substring(delim_index + 1);
                float new_val = 0.0f;
                float.TryParse(val_str, out new_val);
                return new DelayedFieldFloat(new_val, format_str);
            }
        }
    }
}