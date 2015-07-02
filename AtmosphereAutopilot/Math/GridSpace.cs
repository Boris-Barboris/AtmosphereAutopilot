using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace AtmosphereAutopilot
{

    using Vector = VectorArray.Vector;

    /// <summary>
    /// Class-container for sparse spatial data with uniform distribution
    /// </summary>
    /// <typeparam name="T">Type of data, stored in a grid cell</typeparam>
    public class GridSpace<T> where T : struct
    {
        int dim_count;                      // dimension count
        int[] cell_count;                   // amount of cells by dimension

        double[] lower_limits;
        double[] upper_limits;
        double[] cell_size;                 // sizes of cells
        double[] region_size;               // sizes of supercells

        int storage_length;                 // length of main linear storage of supercell
        int[] index_weight;                 // weight of each index in space when linearizing it to 1-dimensional array

        List<Supercell> space = new List<Supercell>();
        List<CellValue> linear_form = new List<CellValue>();

        # region InternalTypes

        public class CellValue
        {
            public CellValue(T data)
            {
                this.data = data;
            }

            public Vector coord;
            public T data;
        }

        public Action<CellValue, T, Vector, Vector> put_criteria = defaultPut;

        static void defaultPut(CellValue oldvalue, T newdata, Vector newcoord, Vector cell_center)
        {
            newcoord.DeepCopy(oldvalue.coord);
            oldvalue.data = newdata;
        }

        // Space will be consisting of supercells, each one is divided according to cell_count
        // and has dimensions according to (region_size). Only one supercell is initially created
        // and under normal conditions we won't need more, but if GridSpace user input will be out of bounds we'll need to
        // allocate additional supercells.
        class Supercell
        {
            GridSpace<T> owner;
            public int[] super_index;
            double[] sc_lower;

            public CellValue[] storage;             // linear storage
            public VectorArray coord_storage;       // coordinate storage

            public Supercell(GridSpace<T> creator, params int[] supercell_index)
            {
                owner = creator;
                super_index = new int[creator.dim_count];
                supercell_index.CopyTo(super_index, 0);
                // get supercell dimensions
                sc_lower = new double[owner.dim_count];
                for (int i = 0; i < owner.dim_count; i++)
                    sc_lower[i] = owner.lower_limits[i] + super_index[i] * owner.region_size[i];
                // allocate storage
                storage = new CellValue[owner.storage_length];
                coord_storage = new VectorArray(creator.dim_count, owner.storage_length);
                cell_center = new Vector(owner.dim_count);
            }

            Vector cell_center;

            public void Put(T data, Vector coord)
            {
                int index = getLinearIndex(coord, cell_center);
                if (storage[index] == null)
                {
                    storage[index] = new CellValue(data);
                    storage[index].coord = coord_storage[index];
                    coord.DeepCopy(storage[index].coord);
                    owner.linear_form.Add(storage[index]);
                }
                else
                {
                    owner.put_criteria(storage[index], data, coord, cell_center);
                }
            }

			public CellValue Get(Vector coord)
            {
                int index = getLinearIndex(coord);
				return storage[index];
            }

            public bool Remove(CellValue val)
            {
                int index = Array.IndexOf(storage, val);
                if (index == -1)
                    return false;
                else
                {
                    storage[index] = null;
                    owner.linear_form.Remove(val);
                    return true;
                }
            }

            /// <summary>
			/// get one-dimensional index of cell from coordinate vector
            /// </summary>
            public int getLinearIndex(Vector coord)
            {
                int linear_index = 0;
                for (int i = 0; i < owner.dim_count; i++)
                {
                    int dim_index = getCellProjection(i, coord[i]);
                    linear_index += dim_index * owner.index_weight[i];
                }
                return linear_index;
            }

            /// <summary>
            /// get one-dimensional index of cell from coordinate vector
            /// </summary>
            public int getLinearIndex(Vector coord, Vector cell_center)
            {
                int linear_index = 0;
                for (int i = 0; i < owner.dim_count; i++)
                {
                    int dim_index = getCellProjection(i, coord[i]);
                    linear_index += dim_index * owner.index_weight[i];
                    cell_center[i] = sc_lower[i] + (dim_index + 0.5) * owner.cell_size[i];
                }
                return linear_index;
            }

            // get cell index of coord over dim'th dimension
            int getCellProjection(int dim, double coord)
            {
                int cell = (int)Math.Floor(
                    (coord - sc_lower[dim]) / owner.cell_size[dim]);
                return cell;
            }            
        }

        #endregion


        /// <summary>
        /// Create new instance of GridSpace
        /// </summary>
        /// <param name="dimensions">number of dimensions</param>
        /// <param name="cells">number of cells for each dimension</param>
        /// <param name="l_cell">lower cell center</param>
        /// <param name="u_cell">upper cell center</param>
        public GridSpace(int dimensions, int[] cells, double[] l_cell, double[] u_cell)
        {
            this.dim_count = dimensions;
            cell_count = cells;
            lower_limits = l_cell;
            upper_limits = u_cell;
            // compute cell dimensions
            cell_size = new double[dim_count];
            for (int i = 0; i < dim_count; i++)
                cell_size[i] = (upper_limits[i] - lower_limits[i]) / (double)(cell_count[i] - 1);
            // recompute limits
            for (int i = 0; i < dim_count; i++)
            {
                lower_limits[i] -= cell_size[i] / 2.0;
                upper_limits[i] += cell_size[i] / 2.0;
            }
            // compute supercell dimensions
            region_size = new double[dim_count];
            for (int i = 0; i < dim_count; i++)
                region_size[i] = upper_limits[i] - lower_limits[i];
            // compute required storage
            storage_length = cells[0];
            for (int i = 1; i < dimensions; i++)
                storage_length *= cells[i];            
            // Prechached index weights for faster linearization
            index_weight = new int[dim_count];
            index_weight[dim_count - 1] = 1;
            for (int i = dim_count - 2; i >= 0; i--)
                index_weight[i] = index_weight[i + 1] * cell_count[i + 1];
            // Initialize base supercell
            space.Add(new Supercell(this, new int[dim_count]));
            // misc
            scindex = new int[dimensions];
        }

        /// <summary>
        /// Put new data point into space
        /// </summary>
        /// <param name="data">Data to put</param>
        /// <param name="coord">coordinate of new data point</param>
        public void Put(T data, Vector coord)
        {
            Supercell scell = GetSupercell(coord);
            scell.Put(data, coord);
        }

        public CellValue Get(Vector coord)
        {
            Supercell scell = GetSupercell(coord, false);
			if (scell == null)
				return null;
			else
				return scell.Get(coord);
        }

        public int getCellIdForCoord(Vector coord)
        {
            Supercell scell = GetSupercell(coord, false);
            if (scell == null)
                return -1;
            else
            {
                int index = space.IndexOf(scell) * storage_length;
                index += scell.getLinearIndex(coord);
                return index;
            }
        }

        public bool Remove(CellValue val)
        {
            Supercell scell = GetSupercell(val.coord, false);
            if (scell == null)
                return false;
            else
                return scell.Remove(val);
        }

        int[] scindex;

        Supercell GetSupercell(Vector coord, bool create = true)
        {
            getSupercellCoord(coord, ref scindex);
			Supercell sc = space.Find((s) => { return s.super_index.SequenceEqual(scindex); });
            if (sc == null && create)
            {
                // need to create new supercell
                sc = new Supercell(this, scindex);
                space.Add(sc);
            }
            return sc;
        }

        void getSupercellCoord(Vector coord, ref int[] output)
        {
            for (int i = 0; i < dim_count; i++)
                output[i] = (int)Math.Floor((coord[i] - lower_limits[i]) / region_size[i]);
        }

        public List<CellValue> Linearized
        {
            get { return linear_form; }
        }

    }

}
