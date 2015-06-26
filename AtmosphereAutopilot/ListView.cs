using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace System.Collections.Generic
{
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

        public void Add(T item) { throw new NotSupportedException("ListView is read-only"); }

        public void Clear() { throw new NotSupportedException("ListView is read-only"); }

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

        public bool Remove(T item) { throw new NotSupportedException("ListView is read-only"); }

        public T this[int index]
        {
            get
            {
                if (lists.Count <= 0)
                    throw new InvalidOperationException("ListView is empty");
                int i = 0;
                while (lists[i].Count <= index)
                {
                    index -= lists[i].Count;
                    i++;
                    if (i >= lists.Count)
                        throw new InvalidOperationException("index too large");
                }
                return lists[i][index];
            }
            set
            {
                if (lists.Count <= 0)
                    throw new InvalidOperationException("ListView is empty");
                int i = 0;
                while (lists[i].Count <= index)
                {
                    index -= lists[i].Count;
                    i++;
                    if (i >= lists.Count)
                        throw new InvalidOperationException("index too large");
                }
                lists[i][index] = value;
            }
        }

        public int IndexOf(T item)
        {
            var comp = EqualityComparer<T>.Default;
            for (int i = 0; i < Count; i++)
                if (comp.Equals(this[i], item))
                    return i;
            return -1;
        }

        public void Insert(int index, T item) { throw new NotSupportedException("ListView is read-only"); }

        public void RemoveAt(int index) { throw new NotSupportedException("ListView is read-only"); }

        public override string ToString()
        {
            string result = "{" + string.Join(", ", this.Select(x => x.ToString()).ToArray()) + "}";
            return result;
        }
    }
}
