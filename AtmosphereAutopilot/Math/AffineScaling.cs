using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace AtmosphereAutopilot
{

    /// <summary>
    /// Linear programming solver
    /// </summary>
    public class AffineScaling
    {
        public Matrix A, b, c, x;

        const double MIN_PARAM_VALUE = 1e-4;

        public void solve(double tolerance, double step_size = 0.35, int iter_limit = 100)
        {
            if (!checkInitPoint(tolerance, x))
            {
                prepareInitData();
                while (iter_limit > 0)
                {
                    IterationResult res = iterate_internal(Ainit, cinit, xinit, tolerance, step_size);
                    iter_limit--;
                    if (res == IterationResult.Unbounded)
                        return;
                    if (res == IterationResult.Optimal/* || checkInitPoint(tolerance, xinit)*/)
                        break;
                }
                update_x();
            }
            while (iter_limit > 0)
            {
                IterationResult res = iterate_internal(A, c, x, tolerance, 0.9);
                iter_limit--;
                if (res != IterationResult.Descended)
                    return;
            }
        }

        Matrix Ainit, vinit, xinit, cinit;

        public AffineScaling(int var_count, int constraint_count)
        {
            init(var_count, constraint_count);
        }

        public void init(int var_count, int constraint_count)
        {
            Matrix.Realloc(constraint_count, var_count, ref A);
            Matrix.Realloc(constraint_count, 1, ref b);
            Matrix.Realloc(var_count, 1, ref c);
            Matrix.Realloc(var_count, 1, ref x);

            Matrix.Realloc(constraint_count, var_count + 1, ref Ainit);
            Matrix.Realloc(constraint_count, 1, ref vinit);
            Matrix.Realloc(var_count + 1, 1, ref xinit);
            Matrix.Realloc(var_count + 1, 1, ref cinit);
        }

        enum IterationResult
        {
            Optimal,
            Descended,
            Unbounded
        }

        Matrix AD2, AD2A, At, AD2Ainv, AD2AinvA, w, r;

        IterationResult iterate_internal(Matrix A, Matrix c, Matrix x, double eps, double beta)
        {
            Matrix.Realloc(A.cols, A.rows, ref At);
            Matrix.Realloc(A.rows, A.cols, ref AD2);
            Matrix.Realloc(A.rows, A.rows, ref AD2A);
            Matrix.Realloc(A.rows, A.rows, ref AD2Ainv);
            Matrix.Realloc(A.rows, A.cols, ref AD2AinvA);
            Matrix.Realloc(A.rows, 1, ref w);
            Matrix.Realloc(c.rows, 1, ref r);

            // calculate A*D2
            for (int i = 0; i < A.rows; i++)
                for (int j = 0; j < A.cols; j++)
                    AD2[i, j] = A[i, j] * (x[j, 0] * x[j, 0]);

            Matrix.TransposeUnsafe(A, At);

            // calculate A*D2*At
            Matrix.MultiplyUnsafe(AD2, At, AD2A);

            // now inverse
            AD2A.Invert(ref AD2Ainv);

            Matrix.MultiplyUnsafe(AD2Ainv, A, AD2AinvA);

            // calculate w
            for (int i = 0; i < A.rows; i++)
                for (int j = 0; j < A.cols; j++)
                {
                    AD2AinvA[i, j] *= x[j, 0] * x[j, 0];
                    w[i, 0] += AD2AinvA[i, j] * c[j, 0];
                }

            // calculate r
            for (int i = 0; i < c.rows; i++)
            {
                r[i, 0] = c[i, 0];
                for (int j = 0; j < A.rows; j++)
                    r[i, 0] -= A[j, i] * w[j, 0];
            }

            double length = 0.0;
            double opt_sum = 0.0;
            bool unbounded = true;
            bool optimal = true;
            for (int i = 0; i < c.rows; i++)
            {
                double m = r[i, 0] * x[i, 0];
                opt_sum += m;
                length += m * m;
                unbounded &= m * x[i, 0] < 0;
                optimal &= r[i, 0] >= 0.0;
            }
            optimal &= (opt_sum <= eps);
            if (unbounded)
                return IterationResult.Unbounded;
            if (optimal)
                return IterationResult.Optimal;

            // update
            length = Math.Sqrt(length);
            if (length > 0.0)
            {
                for (int i = 0; i < c.rows; i++)
                    x[i, 0] = x[i, 0] - beta * (x[i, 0] * x[i, 0]) * r[i, 0] / length;
            }
            return IterationResult.Descended;
        }

        bool checkInitPoint(double tolerance, Matrix x)
        {
            bool feasible = true;
            Matrix.MultiplyUnsafe(A, x, vinit);
            for (int i = 0; i < A.rows; i++)
            {
                vinit[i, 0] = b[i, 0] - vinit[i, 0];
                if (Math.Abs(vinit[i, 0]) >= tolerance)
                    feasible = false;
            }
            if (feasible)
                return true;
            return false;
        }

        void prepareInitData()
        {
            // fill Ainit, xinit and cinit
            for (int i = 0; i < A.rows; i++)
            {
                for (int j = 0; j < A.cols; j++)
                    Ainit[i, j] = A[i, j];
                Ainit[i, A.cols] = vinit[i, 0];
            }

            for (int i = 0; i < c.rows; i++)
            {
                xinit[i, 0] = x[i, 0];
                cinit[i, 0] = 0.0;
            }
            xinit[c.rows, 0] = 1.0;
            cinit[c.rows, 0] = 1.0;
        }

        public void update_x()
        {
            for (int i = 0; i < x.rows; i++)
                x[i, 0] = xinit[i, 0];
        }
    }
}
