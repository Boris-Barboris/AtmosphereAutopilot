/*
Atmosphere Autopilot, plugin for Kerbal Space Program.
Copyright (C) 2015-2017, Baranin Alexander aka Boris-Barboris.
 
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

namespace AtmosphereAutopilot
{
    
    public class MultidimGrid4<T>
    {
        public readonly T[] data;
        float[][] axis;
        int[] sizes = new int[4];

        public MultidimGrid4(float[][] axis, T[] data)
        {
            this.axis = axis;
            this.data = data;
            sizes[3] = 1;
            for (int i = 2; i >= 0; i--)
                sizes[i] = axis[i + 1].Length * sizes[i + 1];
        }

        public T get(int index0, int index1, int index2, int index3)
        {
            int lin_index = 0;
            lin_index += sizes[0] * index0;
            lin_index += sizes[1] * index1;
            lin_index += sizes[2] * index2;
            lin_index += index3;
            return data[lin_index];
        }

        public T getClosest(float coord0, float coord1, float coord2, float coord3, int[] lookup_cache)
        {
            lookup_cache[0] = search_axis(axis[0], coord0, lookup_cache[0]);
            lookup_cache[1] = search_axis(axis[1], coord1, lookup_cache[1]);
            lookup_cache[2] = search_axis(axis[2], coord2, lookup_cache[2]);
            lookup_cache[3] = search_axis(axis[3], coord3, lookup_cache[3]);
            return this.get(lookup_cache[0], lookup_cache[1], lookup_cache[2], lookup_cache[3]);
        }

        int search_axis(float[] axis, float point, int start)
        {
            if (start == 0 && point <= axis[start])
                return start;
            if (start == (axis.Length - 1) && point >= axis[start])
                return start;
            if (point == axis[start])
                return start;
            float min_diff = Math.Abs(axis[start] - point);
            int dir = 0;
            while (true)
            {
                int new_dir = 0;
                if (point > axis[start])
                    new_dir = 1;
                else
                    new_dir = -1;
                if (new_dir * dir < 0)
                    return start;
                float new_diff = Math.Abs(axis[start + new_dir] - point);
                if (min_diff > new_diff)
                {
                    start += new_dir;
                    if (start == 0 || start == (axis.Length - 1))
                        return start;
                    min_diff = new_diff;
                    dir = new_dir;
                }
                else
                    return start;
            }
        }
    }

}

