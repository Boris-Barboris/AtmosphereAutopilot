using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace AtmosphereAutopilot
{

    /// <summary>
    /// Fast matrix-like container, designed for array-like storage of small vectors of doubles.
    /// </summary>
    public class VectorArray
    {
        int vector_size;
        int capacity;
        int data_size;

        public VectorArray(int vsize, int capacity)
        {
            if (vsize < 1)
                throw new ArgumentOutOfRangeException("vsize", "vsize < 1");
            if (capacity < 1)
                throw new ArgumentOutOfRangeException("capacity", "capacity < 1");
            vector_size = vsize;
            this.capacity = capacity;
            data_size = vector_size * capacity;
            data = new double[data_size];
        }

        double[] data;

        public Vector this[int index]
        {
            get { return new Vector(this, index); }
            set { value.DeepCopy(this[index]); }
        }

        public void Resize(int new_capacity)
        {
            if (new_capacity < 1)
                throw new ArgumentOutOfRangeException("new_capacity", "new_capacity < 1");
            capacity = new_capacity;
            data_size = vector_size * capacity;
            Array.Resize(ref data, data_size);
        }

        public struct Vector
        {
            VectorArray source;
            int index;

            public Vector(int size)
            {
                source = new VectorArray(size, 1);
                index = 0;
            }

            public Vector(VectorArray binding, int index)
            {
                source = binding;
                this.index = index;
            }

			public static implicit operator Vector(double[] arr)
			{
				int dim = arr.Length;
				VectorArray a = new VectorArray(dim, 1);
				Vector v = a[0];
				for (int i = 0; i < dim; i++)
					v[i] = arr[i];
				return v;
			}

			public static implicit operator Vector(double val)
			{
				VectorArray a = new VectorArray(1, 1);
				Vector v = a[0];
				v[0] = val;
				return v;
			}

			public double this[int proj]
            {
                get { return source.data[index * source.vector_size + proj]; }
                set { source.data[index * source.vector_size + proj] = value; }
            }

            public void DeepCopy(Vector dest)
            {
                for (int i = 0; i < source.vector_size; i++)
                    dest[i] = this[i];
            }

            public override string ToString()
            {
                if (source == null)
                    return "{}";
                else
                {
                    if (source.data_size == 1)
                        return this[0].ToString("G6");
                    else
                    {
                        string result = "{";
                        for (int i = 0; i < source.vector_size; i++)
                            if (i != source.vector_size - 1)
                                result += this[i].ToString("G6") + ", ";
                            else
                                result += this[i].ToString("G6");
                        result += "}";
                        return result;
                    }
                }
            }

            public override bool Equals(Object obj)
            {
                return obj is Vector && this == (Vector)obj;
            }
            public override int GetHashCode()
            {
                return source.GetHashCode() ^ index.GetHashCode();
            }
            public static bool operator ==(Vector x, Vector y)
            {
                return x.source == y.source && x.index == y.index;
            }
            public static bool operator !=(Vector x, Vector y)
            {
                return !(x == y);
            }

            public Vector Scale(double s)
            {
                for (int i = 0; i < source.vector_size; i++)
                    this[i] *= s;
                return this;
            }
        }
    }
    
}
