using System;
using System.Collections.Generic;

namespace AtmosphereAutopilot
{
	class GradientDescend
	{
		public GradientDescend(int param_count)
		{
			this.param_count = param_count;
			changed_param1 = new double[param_count];
			changed_param2 = new double[param_count];
			gradient = new double[param_count];
		}

		int param_count;
		double[] changed_param1, changed_param2;
		double[] gradient;

		public int max_func_calls = 100;
		public double descend_k = 0.0;
		public double descend_miss_k = 0.1;
		
		const double PROBE_DX = 0.1;

		public void apply(double[] parameters, Func<double[], double> function, ref int call_counter)
		{
			bool skip_gradient = false;
			double f_val = 0.0;
			double sqrnorm = 0.0;

			parameters.CopyTo(changed_param1, 0);
			while (call_counter < max_func_calls)
			{
				if (!skip_gradient)
				{
					f_val = function(changed_param1);
					// get gradient
					for (int i = 0; i < param_count; i++)
					{
						double orig_ch = changed_param1[i];
						changed_param1[i] += PROBE_DX;
						gradient[i] = (function(changed_param1) - f_val) / PROBE_DX;
						changed_param1[i] = orig_ch;
					}

					// get gradient size
					sqrnorm = 0.0;
					for (int i = 0; i < param_count; i++)
						sqrnorm += gradient[i] * gradient[i];
				}
				skip_gradient = false;
				
				// evaluate descend speed
				double descend_speed = f_val * (1 - descend_k) / sqrnorm;
				
				// apply descend
				for (int i = 0; i < param_count; i++)
					changed_param2[i] = changed_param1[i] - gradient[i] * descend_speed;
				double f_val_new = function(changed_param2);

				if (f_val_new >= f_val)
				{
					// we've fucked up
					descend_k = 1 - descend_miss_k * (1 - descend_k);
					skip_gradient = true;
				}
				else
				{
					changed_param2.CopyTo(changed_param1, 0);
				}
			}

			// output new optimization
			changed_param1.CopyTo(parameters, 0);
		}
	}
}
