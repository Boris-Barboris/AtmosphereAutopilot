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
using System.Linq;
using System.Text;

namespace System.Collections.Generic
{

    // IList concatenator
    public class ListView<T> : IList<T>, ICollection<T>, IEnumerable<T>, IEnumerable
    {
        List<IList<T>> lists = new List<IList<T>>();

        public ListView(params IList<T>[] targets)
        {
            foreach (var target in targets)
                if (target != null)
                    lists.Add(target);
        }

        public IEnumerator<T> GetEnumerator()
        {
            foreach (var list in lists)
                foreach (T x in list)
                    yield return x;
        }

        IEnumerator IEnumerable.GetEnumerator()
        {
            return GetEnumerator();
        }

        public int Count
        {
            get
            {
                int count = 0;
                foreach (var l in lists)
                    count += l.Count;
                return count;
            }
        }

        public bool IsReadOnly { get { return false; } }

        public void Add(T item) { throw new NotSupportedException("ListView is fixed-size"); }

        public void Clear() { throw new NotSupportedException("ListView is fixed-size"); }

        public bool Contains(T item)
        {
            var comp = EqualityComparer<T>.Default;
            for (int i = 0; i < Count; i++)
                if (comp.Equals(this[i], item))
                    return true;
            return false;
        }

        public void CopyTo(T[] array, int arrayIndex)
        {
            for (int i = arrayIndex, j = 0; j < Count; j++, i++)
                array[i] = this[j];
        }

        public bool Remove(T item) { throw new NotSupportedException("ListView is fixed-size"); }

        public T this[int index]
        {
            get
            {
                int i = get_list_index(ref index);
                return lists[i][index];
            }
            set
            {
                int i = get_list_index(ref index);
                lists[i][index] = value;
            }
        }

        int prev_query = int.MaxValue;
        int prev_list_index = 0;
        int prev_index_shift = 0;

        int get_list_index(ref int lin_index)
        {
            if (lists.Count == 0)
                throw new InvalidOperationException("ListView is empty");
            int i = 0;
            if (lin_index < prev_query)
            {
                prev_query = lin_index;
                prev_index_shift = 0;
                while (lists[i].Count <= lin_index)
                {
                    prev_index_shift += lists[i].Count;
                    lin_index -= lists[i].Count;
                    i++;
                    if (i >= lists.Count)
                        throw new InvalidOperationException("index too large");
                }
            }
            else
            {
                i = prev_list_index;
                prev_query = lin_index;
                lin_index -= prev_index_shift;
                while (lists[i].Count <= lin_index)
                {
                    prev_index_shift += lists[i].Count;
                    lin_index -= lists[i].Count;
                    i++;
                    if (i >= lists.Count)
                        throw new InvalidOperationException("index too large");
                }
            }
            prev_list_index = i;
            return i;
        }

        public void reset_cached_index()
        {
            prev_query = int.MaxValue;
            prev_list_index = 0;
            prev_index_shift = 0;
        }

        public int IndexOf(T item)
        {
            var comp = EqualityComparer<T>.Default;
            for (int i = 0; i < Count; i++)
                if (comp.Equals(this[i], item))
                    return i;
            return -1;
        }

        public void Insert(int index, T item) { throw new NotSupportedException("ListView is fixed-size"); }

        public void RemoveAt(int index) { throw new NotSupportedException("ListView is fixed-size"); }

        public override string ToString()
        {
            string result = "{" + string.Join(", ", this.Select(x => x.ToString()).ToArray()) + "}";
            return result;
        }
    }

}
