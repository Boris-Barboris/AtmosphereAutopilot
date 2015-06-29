using System;
using System.Collections.Generic;
using System.Collections;
using System.Linq;
using System.Text;

namespace AtmosphereAutopilot
{
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
