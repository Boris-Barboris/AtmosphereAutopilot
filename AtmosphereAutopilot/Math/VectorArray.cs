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

        public double[] data;

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

            public string ToString(string format)
            {
                if (source == null)
                    return "{}";
                else
                {
                    if (source.data_size == 1)
                        return this[0].ToString(format);
                    else
                    {
                        string result = "{";
                        for (int i = 0; i < source.vector_size; i++)
                            if (i != source.vector_size - 1)
                                result += this[i].ToString(format) + ", ";
                            else
                                result += this[i].ToString(format);
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

            public Vector InverseScale(IList<double> s)
            {
                for (int i = 0; i < source.vector_size; i++)
                    this[i] /= s[i];
                return this;
            }

            public static double SqrLength(Vector a, Vector b)
            {
                double res = 0.0;
                for (int i = 0; i < a.source.vector_size; i++)
                {
                    double proj = a[i] - b[i];
                    res += proj * proj;
                }
                return res;
            }

            public static void Add(Vector a, Vector b, Vector res)
            {
                for (int i = 0; i < a.source.vector_size; i++)
                    res[i] = a[i] + b[i];
            }

            public static void Sub(Vector a, Vector b, Vector res)
            {
                for (int i = 0; i < a.source.vector_size; i++)
                    res[i] = a[i] - b[i];
            }

            public double SqrLength()
            {
                double res = 0.0;
                for (int i = 0; i < source.vector_size; i++)
                {
                    double proj = this[i];
                    res += proj * proj;
                }
                return res;
            }

            public static double Length(Vector a, Vector b)
            {
                return Math.Sqrt(SqrLength(a, b));
            }
        }
    }
    
}
