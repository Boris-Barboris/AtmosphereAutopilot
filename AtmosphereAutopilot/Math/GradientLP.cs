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

        public void solve(double tolerance, double constraint_step, double optimization_step, int iter_limit = 100)
        {
            double old_goal = goal_function();
            while (iter_limit > 0)
            {
                descend(constraint_step, optimization_step);
                double new_goal = goal_function();
                if (Math.Abs(new_goal - old_goal) / Math.Max(1e-6, old_goal) < tolerance)
                    break;
                old_goal = new_goal;
                iter_limit--;
            }
        }

        Matrix cons_grad, opt_grad, grad, x_new;

        void descend(double cons_k, double opt_k)
        {
            Matrix.Realloc(x.rows, 1, ref cons_grad);
            Matrix.Realloc(x.rows, 1, ref opt_grad);
            Matrix.Realloc(x.rows, 1, ref grad);
            Matrix.Realloc(x.rows, 1, ref x_new);

            // calculate constraint gradients
            for (int i = 0; i < A.rows; i++)
            {
                double cons_err = 0.0;
                for (int j = 0; j < x.rows; j++)
                    cons_err += A[i, j] * x[j, 0];
                for (int j = 0; j < x.rows; j++)
                    cons_grad[j, 0] += 2.0 * (cons_err - b[i, 0]) * A[i, j];
            }

            // calculate optimizing gradient
            for (int i = 0; i < x.rows; i++)
                opt_grad[i, 0] = -opt_k * c[i, 0];

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

            for (int i = 0; i < x.rows; i++)
                grad[i, 0] = cons_k * cons_grad[i, 0] + opt_grad[i, 0];

            // descend
            for (int i = 0; i < x.rows; i++)
                x_new[i, 0] = x[i, 0] - grad[i, 0];
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

        double goal_function()
        {
            double result = 0.0;
            for (int i = 0; i < x.rows; i++)
                result += x[i, 0] * c[i, 0];
            return result;
        }

    }
}
