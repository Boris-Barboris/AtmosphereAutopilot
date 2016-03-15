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
using System.Collections;
using System.Linq;
using System.Text;

namespace AtmosphereAutopilot
{

    // IList adapter
    public class ListSelector<ST, RT> : IList<RT>, ICollection<RT>, IEnumerable<RT>, IEnumerable
    {
        IList<ST> source;
        Func<ST, RT> selector;

        public ListSelector(IList<ST> source, Func<ST, RT> selector)
        {
            this.source = source;
            this.selector = selector;
        }

        public IEnumerator<RT> GetEnumerator()
        {
            foreach (var x in source)
                yield return selector(x);
        }

        IEnumerator IEnumerable.GetEnumerator()
        {
            return GetEnumerator();
        }

        public int Count
        {
            get
            {
                return source.Count;
            }
        }

        public bool IsReadOnly { get { return true; } }

        public void Add(RT item) { throw new NotSupportedException("ListSelector is fixed-size"); }

        public void Clear() { throw new NotSupportedException("ListSelector is fixed-size"); }

        public bool Contains(RT item)
        {
            var comp = EqualityComparer<RT>.Default;
            for (int i = 0; i < Count; i++)
                if (comp.Equals(this[i], item))
                    return true;
            return false;
        }

        public void CopyTo(RT[] array, int arrayIndex)
        {
            for (int i = arrayIndex, j = 0; j < Count; j++, i++)
                array[i] = this[j];
        }

        public bool Remove(RT item) { throw new NotSupportedException("ListSelector is fixed-size"); }

        public RT this[int index]
        {
            get
            {
                return selector(source[index]);
            }
            set
            {
                throw new NotSupportedException("ListSelector is read-only");
            }
        }

        public int IndexOf(RT item)
        {
            var comp = EqualityComparer<RT>.Default;
            for (int i = 0; i < Count; i++)
                if (comp.Equals(this[i], item))
                    return i;
            return -1;
        }

        public void Insert(int index, RT item) { throw new NotSupportedException("ListSelector is fixed-size"); }

        public void RemoveAt(int index) { throw new NotSupportedException("ListSelector is fixed-size"); }

        public override string ToString()
        {
            string result = "{" + string.Join(", ", this.Select(x => x.ToString()).ToArray()) + "}";
            return result;
        }

    }
}
