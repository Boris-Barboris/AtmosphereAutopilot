using System;
using System.Collections.Generic;

namespace AtmosphereAutopilot
{
	public class GradientDescent
	{
		public GradientDescent(int param_count)
		{
			this.param_count = param_count;
			partial_deriv = new float[param_count];
			changed_param = new float[param_count];
		}

		int param_count;
		float[] partial_deriv;
		float[] changed_param;

		public float[] apply(float[] parameters, Func<float[], float> function, float[] probe_delta, float[] descent_k)
		{			
			parameters.CopyTo(changed_param, 0);
			float func_value = function(parameters);
			for (int i = 0; i < param_count; i++)
			{
				float old_param = parameters[i];
				if (Math.Abs(probe_delta[i]) < 1e-10f)
					probe_delta[i] = 1e-10f;
				changed_param[i] = parameters[i] + probe_delta[i];
				float delta_value = function(changed_param) - func_value;				
				partial_deriv[i] = delta_value / probe_delta[i];
				changed_param[i] = old_param;
			}
			for (int i = 0; i < param_count; i++)
				parameters[i] = parameters[i] - descent_k[i] * partial_deriv[i];

			return parameters;
		}
	}
}
