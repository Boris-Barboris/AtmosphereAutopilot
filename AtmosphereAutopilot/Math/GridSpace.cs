﻿using System;
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

        double[] lower_limits;              // lower bounds of supercell
        double[] upper_limits;              // upper bounds of supercell
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

        // Space will be consisting of supercells, each one is divided according to cell_count
        // and has dimensions according to (upper_limits - lower_limits). Only one supercell is initially created
        // and under normal conditions we won't need more, but if GridSpace user input will be out of bounds we'll need to
        // allocate additional supercells.
        class Supercell
        {
            GridSpace<T> owner;
            public int[] super_index;

            public CellValue[] storage;             // linear storage
            public VectorArray coord_storage;       // coordinate storage

            public Supercell(GridSpace<T> creator, params int[] supercell_index)
            {
                owner = creator;
                super_index = supercell_index;
                // allocate storage
                storage = new CellValue[owner.storage_length];
                coord_storage = new VectorArray(creator.dim_count, owner.storage_length);
            }

            public void Put(T data, Vector coord)
            {
                int index = getLinearIndex(coord);
                if (storage[index] == null)
                {
                    storage[index] = new CellValue(data);
                    storage[index].coord = coord_storage[index];
                    coord.DeepCopy(storage[index].coord);
                    owner.linear_form.Add(storage[index]);
                }
                else
                {
                    coord.DeepCopy(storage[index].coord);
                    storage[index].data = data;
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

            // get cell index of coord over dim'th dimension
            int getCellProjection(int dim, double coord)
            {
                int cell = (int)Math.Floor(
                    (coord - (owner.lower_limits[dim] + super_index[dim] * owner.region_size[dim])) / owner.cell_size[dim]);
                return cell;
            }            
        }

        #endregion


        /// <summary>
        /// Create new instance of GridSpace
        /// </summary>
        /// <param name="dimensions">number of dimensions</param>
        /// <param name="cells">number of cells for each dimension</param>
        /// <param name="l_bound">assumed lower bounds for each dimension</param>
        /// <param name="u_bound">assumed upper bounds for each dimension</param>
        public GridSpace(int dimensions, int[] cells, double[] l_bound, double[] u_bound)
        {
            this.dim_count = dimensions;
            cell_count = cells;
            lower_limits = l_bound;
            upper_limits = u_bound;
            // compute cell dimensions
            cell_size = new double[dim_count];
            for (int i = 0; i < dim_count; i++)
                cell_size[i] = (upper_limits[i] - lower_limits[i]) / cell_count[i];
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

        Supercell GetSupercell(Vector coord, bool create = true)
        {
            int[] scindex = getSupercellCoord(coord);
			Supercell sc = space.Find((s) => { return s.super_index.SequenceEqual(scindex); });
            if (sc == null && create)
            {
                // need to create new supercell
                sc = new Supercell(this, scindex);
                space.Add(sc);
            }
            return sc;
        }

        int[] getSupercellCoord(Vector coord)
        {
			int[] output = new int[dim_count];
            for (int i = 0; i < dim_count; i++)
                output[i] = (int)Math.Floor((coord[i] - lower_limits[i]) / region_size[i]);
			return output;
        }

        public List<CellValue> Linearized
        {
            get { return linear_form; }
        }
    }

}