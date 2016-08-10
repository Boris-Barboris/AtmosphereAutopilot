using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace AtmosphereAutopilot
{

    /// <summary>
    /// Euristic gradient descend optimizer for [0..1] bounded parameters in problem:
    /// maximize c*x while A*x=b and x in 0..1
    /// </summary>
    public class GradientLP
    {
        public Matrix A, b, c, x;

        public GradientLP(int param_count, int constraint_count)
        {
            init(param_count, constraint_count);
        }

        public void init(int param_count, int constraint_count)
        {
            Matrix.Realloc(constraint_count, param_count, ref A);
            Matrix.Realloc(constraint_count, 1, ref b);
            Matrix.Realloc(param_count, 1, ref c);
            Matrix.Realloc(param_count, 1, ref x);
        }

        public void solve(double constraint_step, out double start_cons_err, out double end_err, int iter_limit = 100)
        {
            int iter_count = iter_limit;
            double s = 0.0;
            start_cons_err = 0.0;
            double e = 0.0;
            while (iter_limit > 0)
            {                
                descend(constraint_step, out s, out e);
                if (iter_limit == iter_count)
                    start_cons_err = s;
                if (speed == 1e-10)
                    break;
                iter_limit--;
            }
            end_err = e;
        }

        Matrix cons_grad, opt_grad, grad, x_new;

        public double constraint_error = 0.0;
        public double speed = 1e-3;
        public double adapt = 10.0;
        public double sqr_err = 0.0;

        void descend(double cons_k, out double start_err, out double end_err)
        {
            Matrix.Realloc(x.rows, 1, ref cons_grad);
            Matrix.Realloc(x.rows, 1, ref opt_grad);
            Matrix.Realloc(x.rows, 1, ref grad);
            Matrix.Realloc(x.rows, 1, ref x_new);

            // calculate constraint gradient and error
            sqr_err = 0.0;
            for (int i = 0; i < A.rows; i++)
            {
                double cons_err = 0.0;
                for (int j = 0; j < x.rows; j++)
                    cons_err += A[i, j] * x[j, 0];
                for (int j = 0; j < x.rows; j++)
                    cons_grad[j, 0] += 2.0 * cons_k * (cons_err - b[i, 0]) * A[i, j];
                sqr_err += cons_k * (cons_err - b[i, 0]) * (cons_err - b[i, 0]);
            }
            start_err = sqr_err;
            end_err = start_err;

            // calculate optimizing gradient and errors
            for (int i = 0; i < x.rows; i++)
            {
                double terr = c[i, 0] * (x[i, 0] - 10.0) / x.rows;
                sqr_err += terr * terr;
                opt_grad[i, 0] = 2.0 * c[i, 0] * terr / x.rows;
            }

            // account for 0..1 bounding
            for (int i = 0; i < x.rows; i++)
            {
                if (x[i, 0] == 0.0 && cons_grad[i, 0] > 0.0)
                    cons_grad[i, 0] = 0.0;
                if (x[i, 0] == 1.0 && cons_grad[i, 0] < 0.0)
                    cons_grad[i, 0] = 0.0;
                if (x[i, 0] == 0.0 && opt_grad[i, 0] > 0.0)
                    opt_grad[i, 0] = 0.0;
                if (x[i, 0] == 1.0 && opt_grad[i, 0] < 0.0)
                    opt_grad[i, 0] = 0.0;
            }

            // dot check for optimization gradient
            double dot = 0.0;
            for (int i = 0; i < x.rows; i++)
                dot += cons_grad[i, 0] * opt_grad[i, 0];
            if (dot < 0.0)
            {
                double grad_dotdot = 0.0;
                for (int i = 0; i < x.rows; i++)
                    grad_dotdot += cons_grad[i, 0] * cons_grad[i, 0];
                for (int i = 0; i < x.rows; i++)
                    opt_grad[i, 0] -= dot * cons_grad[i, 0] / grad_dotdot;
            }

            // summ gradient components
            for (int i = 0; i < x.rows; i++)
                grad[i, 0] = cons_grad[i, 0] + opt_grad[i, 0];

            int iter = 0;
            while (iter < 20)
            {
                // descend
                for (int i = 0; i < x.rows; i++)
                    x_new[i, 0] = x[i, 0] - speed * grad[i, 0];

                // calculate new error
                double new_error = 0.0;
                for (int i = 0; i < A.rows; i++)
                {
                    // constraints
                    double cons_err = 0.0;
                    for (int j = 0; j < x.rows; j++)
                        cons_err += A[i, j] * x_new[j, 0];
                    new_error += cons_k * (cons_err - b[i, 0]) * (cons_err - b[i, 0]);
                }
                end_err = new_error;
                for (int i = 0; i < x.rows; i++)
                {
                    // optimization error
                    double terr = c[i, 0] * (x_new[i, 0] - 10.0) / x.rows;
                    new_error += terr * terr;
                }

                if (new_error < sqr_err)
                {
                    speed = speed * (1.0 + 0.5 * (adapt - 1.0));
                    break;
                }
                else
                {
                    speed = speed / adapt;
                    if (speed < 1e-10)
                    {
                        speed = 1e-10;
                        end_err = start_err;
                        return;
                    }
                }
                iter++;
            }

            double proportion = 1.0;
            for (int i = 0; i < x.rows; i++)
            {
                if (x_new[i, 0] > 1.0)
                    proportion = Math.Min(proportion, (1.0 - x[i, 0]) / (x_new[i, 0] - x[i, 0]));
                if (x_new[i, 0] < 0.0)
                    proportion = Math.Min(proportion, (0.0 - x[i, 0]) / (x_new[i, 0] - x[i, 0]));
            }
            // clamp to stay in 0..1 region
            for (int i = 0; i < x.rows; i++)
                x[i, 0] += proportion * (x_new[i, 0] - x[i, 0]);
        }

    }
}
