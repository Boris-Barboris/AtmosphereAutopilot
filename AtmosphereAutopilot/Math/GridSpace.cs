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

    using Vector = VectorArray.Vector;

    /// <summary>
    /// Class-container for sparse spatial data with uniform distribution
    /// </summary>
    /// <typeparam name="T">Type of data, stored in a grid cell</typeparam>
    public class GridSpace<T> where T : struct
    {
        int dim_count;                      // dimension count
        int[] cell_count;                   // amount of cells by dimension

        public double[] lower_cell;         // center of lowest cell
        public double[] upper_cell;         // center of highest cell
        double[] lower_border;              // lower region border
        double[] cell_size;                 // sizes of cells

        public readonly int storage_length;     // length of main linear storage of supercell
        int[] index_weight;                 // weight of each index in space when linearizing it to 1-dimensional array

        List<CellValue> linear_form;

        public class CellValue
        {
            public bool empty;
            public Vector coord;
            public T data;
        }

        public delegate void PutCriteria(CellValue oldvalue, T newdata, Vector new_coord, Vector cell_center, double[] cell_sizes);

        public PutCriteria put_method = defaultPutCriteria;

        public static void defaultPutCriteria(CellValue oldvalue, T newdata, Vector new_coord, Vector cell_center, double[] cell_sizes)
        {
            new_coord.DeepCopy(oldvalue.coord);
            oldvalue.data = newdata;
        }

        CellValue[] storage;             // linear storage
        VectorArray coord_storage;       // coordinate storage

        Vector cell_center;

        public void Put(T data, Vector coord)
        {
            int index = getLinearIndex(coord, cell_center);
            if (storage[index].empty)
            {
                storage[index].data = data;
                coord.DeepCopy(storage[index].coord);
                storage[index].empty = false;
                linear_form.Add(storage[index]);
            }
            else
            {
                put_method(storage[index], data, coord, cell_center, cell_size);
            }
        }

        public CellValue Get(Vector coord)
        {
            int index = getLinearIndex(coord);
            return storage[index];
        }

        public bool Remove(CellValue val)
        {
            val.empty = true;
            return linear_form.Remove(val);
        }

        /// <summary>
        /// get one-dimensional index of cell from coordinate vector
        /// </summary>
        public int getLinearIndex(Vector coord)
        {
            int linear_index = 0;
            for (int i = 0; i < dim_count; i++)
            {
                int dim_index = getCellProjection(i, coord[i]);
                linear_index += dim_index * index_weight[i];
            }
            return linear_index;
        }

        /// <summary>
        /// get one-dimensional index of cell from coordinate vector
        /// </summary>
        int getLinearIndex(Vector coord, Vector cell_center)
        {
            int linear_index = 0;
            for (int i = 0; i < dim_count; i++)
            {
                int dim_index = getCellProjection(i, coord[i]);
                linear_index += dim_index * index_weight[i];
                cell_center[i] = dim_index * cell_size[i] + lower_cell[i];
            }
            return linear_index;
        }

        // get cell index of coord over dim'th dimension
        int getCellProjection(int dim, double coord)
        {
            int cell = (int)Math.Floor(
                (coord - lower_border[dim]) / cell_size[dim]);
            return Common.Clamp(cell, 0, cell_count[dim]-1);
        }            

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
            lower_cell = l_cell;
            upper_cell = u_cell;
            // compute dimensions
            cell_size = new double[dim_count];
            lower_border = new double[dim_count];
            recompute_region();
            // compute required storage
            storage_length = cells[0];
            for (int i = 1; i < dimensions; i++)
                storage_length *= cells[i];           
            // Prechached index weights for faster linearization
            index_weight = new int[dim_count];
            index_weight[dim_count - 1] = 1;
            for (int i = dim_count - 2; i >= 0; i--)
                index_weight[i] = index_weight[i + 1] * cell_count[i + 1];
            // allocate storage
            storage = new CellValue[storage_length];
            coord_storage = new VectorArray(dim_count, storage_length);
            cell_center = new Vector(dim_count);
            for (int i = 0; i < storage_length; i++)
            {
                storage[i] = new CellValue();
                storage[i].empty = true;
                storage[i].coord = coord_storage[i];
            }
            linear_form = new List<CellValue>(storage_length);
        }

        /// <summary>
        /// Recompute service dimensions. Call this after manually changing upper_cell or lower_cell
        /// </summary>
        public void recompute_region()
        {
            for (int i = 0; i < dim_count; i++)
            {
                cell_size[i] = (upper_cell[i] - lower_cell[i]) / (double)(cell_count[i] - 1);
                lower_border[i] = lower_cell[i] - cell_size[i] * 0.5;
            }
        }

        public List<CellValue> Linearized
        {
            get { return linear_form; }
        }

    }

}
