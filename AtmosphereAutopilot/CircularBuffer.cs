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

using System.Collections;
using System.Collections.Generic;
using System.Threading;

namespace System.Collections.Generic
{
    public class CircularBufferAA<T> : IList<T>, ICollection<T>, IEnumerable<T>, ICollection, IEnumerable
    {
        private int capacity;       // underlying array size
        private int size;           // current count of elements
        private int head;           // index of first element
        private int tail;           // index of element behind the last one
        private T[] buffer;         // underlying array

        [NonSerialized]
        private object syncRoot;

        public CircularBufferAA(int capacity) : this(capacity, false) { }

        public CircularBufferAA(int capacity, bool allowOverflow)
        {
            if (capacity <= 0)
                throw new ArgumentException("capacity must be greater than zero.",
                    "capacity");

            this.capacity = capacity;
            size = 0;
            head = 0;
            tail = 0;
            buffer = new T[capacity];
            AllowOverflow = allowOverflow;
        }

        public CircularBufferAA(int capacity, bool allowOverflow, T first_value) : this(capacity, allowOverflow)
        {
            Put(first_value);   
        }

        public bool AllowOverflow
        {
            get;
            set;
        }

        public int Capacity
        {
            get { return capacity; }
            set
            {
                if (value == capacity)
                    return;

                if (value < size)
                    throw new ArgumentOutOfRangeException("value",
                        "value must be greater than or equal to the buffer size.");

                var dst = new T[value];
                if (size > 0)
                    CopyTo(dst);
                buffer = dst;

                capacity = value;
            }
        }

        public int Size
        {
            get { return size; }
        }

        public bool Contains(T item)
        {
            int bufferIndex = head;
            var comparer = EqualityComparer<T>.Default;
            for (int i = 0; i < size; i++, bufferIndex++)
            {
                if (bufferIndex == capacity)
                    bufferIndex = 0;

                if (item == null && buffer[bufferIndex] == null)
                    return true;
                else if ((buffer[bufferIndex] != null) &&
                    comparer.Equals(buffer[bufferIndex], item))
                    return true;
            }

            return false;
        }

        public void Clear()
        {
            size = 0;
            head = 0;
            tail = 0;
        }

        public int Put(T[] src)
        {
            return Put(src, 0, src.Length);
        }

        public int Put(T[] src, int offset, int count)
        {
            int realCount = AllowOverflow ? count : Math.Min(count, capacity - size);
            int srcIndex = offset;
            for (int i = 0; i < realCount; i++, srcIndex++)
            {
                if (tail == capacity)
                    tail = 0;
                buffer[tail] = src[srcIndex];
                tail = (tail + 1) % capacity;
            }
            size = Math.Min(size + realCount, capacity);
            return realCount;
        }

        public void Put(T item)
        {
            if (size >= capacity)
                if (!AllowOverflow)
                    throw new IndexOutOfRangeException("Buffer is full.");
                else 
                {
                    buffer[tail] = item;
                    tail = (tail + 1) % capacity;
                    head = tail;
                    return;
                }
            size++;
            buffer[tail] = item;
            tail = (tail + 1) % capacity;
        }

        public T getWritingCell()
        {
            return buffer[tail];
        }

        public T getFromTail(int shift)
        {
            if (shift >= size)
                throw new IndexOutOfRangeException("shift >= size = " + size.ToString());
            return buffer[capacity - 1 - ((capacity - tail + shift) % capacity)];
        }

        public T this[int index]
        {
            get { return buffer[(index + head) % capacity]; }
            set { buffer[(index + head) % capacity] = value; }
        }

        public T getLast()
        {
            if (size <= 0)
                return buffer[0];
            else
                return this[size - 1];
        }

        public void Skip(int count)
        {
            head += count;
            if (head >= capacity)
                head -= capacity;
        }

        public T[] Get(int count)
        {
            var dst = new T[count];
            Get(dst);
            return dst;
        }

        public int Get(T[] dst)
        {
            return Get(dst, 0, dst.Length);
        }

        public int Get(T[] dst, int offset, int count)
        {
            int realCount = Math.Min(count, size);
            int dstIndex = offset;
            for (int i = 0; i < realCount; i++, head++, dstIndex++)
            {
                if (head == capacity)
                    head = 0;
                dst[dstIndex] = buffer[head];
            }
            size -= realCount;
            return realCount;
        }

        public T Get()
        {
            if (size == 0)
                throw new InvalidOperationException("Buffer is empty.");

            var item = buffer[head];
            if (++head == capacity)
                head = 0;
            size--;
            return item;
        }

        public void CopyTo(T[] array)
        {
            CopyTo(array, 0);
        }

        public void CopyTo(T[] array, int arrayIndex)
        {
            CopyTo(0, array, arrayIndex, size);
        }

        public void CopyTo(int index, T[] array, int arrayIndex, int count)
        {
            if (count > size)
                throw new ArgumentOutOfRangeException("count",
                    "count cannot be greater than the buffer size.");

            int bufferIndex = head;
            for (int i = 0; i < count; i++, bufferIndex++, arrayIndex++)
            {
                if (bufferIndex == capacity)
                    bufferIndex = 0;
                array[arrayIndex] = buffer[bufferIndex];
            }
        }

        public IEnumerator<T> GetEnumerator()
        {
            int bufferIndex = head;
            for (int i = 0; i < size; i++, bufferIndex++)
            {
                if (bufferIndex == capacity)
                    bufferIndex = 0;

                yield return buffer[bufferIndex];
            }
        }

        public T[] GetBuffer()
        {
            return buffer;
        }

        public T[] ToArray()
        {
            var dst = new T[size];
            CopyTo(dst);
            return dst;
        }

        # region IList<T> Members

        public int IndexOf(T item)
        {
            int bufferIndex = head;
            var comparer = EqualityComparer<T>.Default;
            for (int i = 0; i < size; i++, bufferIndex++)
            {
                if (bufferIndex == capacity)
                    bufferIndex = 0;
                if (item == null && buffer[bufferIndex] == null)
                    return bufferIndex;
                else if ((buffer[bufferIndex] != null) &&
                    comparer.Equals(buffer[bufferIndex], item))
                    return bufferIndex;
            }
            return -1;
        }

        public void Insert(int index, T item)
        {
            if (index > size || index < 0)
                throw new ArgumentOutOfRangeException("index", "CircularBufferAA Insert at index > size");
            if (index == size)
                Put(item);
            else
            {
                for (int i = 0; i < index; i++)
                    this[i] = this[i + 1];
                this[index] = item;
            }
        }

        public void RemoveAt(int index)
        {
            if (index >= size || index < 0)
                throw new ArgumentOutOfRangeException("index", "CircularBufferAA RemoveAt index >= size");
            if (index != size -1)
            {
                for (int i = index; i < size; i++)
                    this[i] = this[i + 1];
            }
            tail = (tail - 1 + capacity) % capacity;
            size--;
        }

        #endregion

        #region ICollection<T> Members

        int ICollection<T>.Count
        {
            get { return Size; }
        }

        bool ICollection<T>.IsReadOnly
        {
            get { return false; }
        }

        void ICollection<T>.Add(T item)
        {
            Put(item);
        }

        bool ICollection<T>.Remove(T item)
        {
            if (size == 0)
                return false;

            Get();
            return true;
        }

        #endregion

        #region IEnumerable<T> Members

        IEnumerator<T> IEnumerable<T>.GetEnumerator()
        {
            return GetEnumerator();
        }

        #endregion

        #region ICollection Members

        int ICollection.Count
        {
            get { return Size; }
        }

        bool ICollection.IsSynchronized
        {
            get { return false; }
        }

        object ICollection.SyncRoot
        {
            get
            {
                if (syncRoot == null)
                    Interlocked.CompareExchange(ref syncRoot, new object(), null);
                return syncRoot;
            }
        }

        void ICollection.CopyTo(Array array, int arrayIndex)
        {
            CopyTo((T[])array, arrayIndex);
        }

        #endregion

        #region IEnumerable Members

        IEnumerator IEnumerable.GetEnumerator()
        {
            return (IEnumerator)GetEnumerator();
        }

        #endregion
    }
}
