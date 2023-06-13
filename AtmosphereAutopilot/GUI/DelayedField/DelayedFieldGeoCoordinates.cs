/*
Atmosphere Autopilot, plugin for Kerbal Space Program.
Copyright (C) 2015-2016, Baranin Alexander aka Boris-Barboris.
Copyright (C) 2016, George Sedov.

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

using UnityEngine;

namespace AtmosphereAutopilot
{
    /// <summary>
    /// Struct for organizing delayed textfield input. Use DisplayLayout() to integrate into OnGUI code,
    /// use OnUpdate() to account for time.
    /// </summary>
    public class DelayedFieldGeoCoordinates : DelayedFieldFloat
    {
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
        public DelayedFieldGeoCoordinates(
            float init_value,
            string format,
            CoordFormat coord_fmt = CoordFormat.Decimal) : base(init_value, format)
        {
            coord_format = coord_fmt;
        }

        public CoordFormat check_coord_format()
        {
            return coord_format;
        }

        public new void OnUpdate()
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
							Log.info("[AtmosphereAutopilot] coords {0:F4}", val);
							return;
						}
						if (dms.Length > 1) return;	// wait for NS/EW
					}
				}
                {
                    float v;
                    if (float.TryParse(input_str, out v))
                        this.val = v;
                    else
                        this.input_str = this.val.ToString(format_str);
                }
            }
        }
    }
}
