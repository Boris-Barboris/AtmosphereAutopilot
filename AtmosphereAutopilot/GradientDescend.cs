using System;
using System.Collections.Generic;

namespace AtmosphereAutopilot
{
	class GradientDescend
	{
		public GradientDescend(int param_count)
		{
			this.param_count = param_count;
			partial_deriv = new float[param_count];
			changed_param1 = new float[param_count];
		}

		int param_count;
		float[] partial_deriv;
		float[] changed_param1;

		public float[] apply(float[] parameters, Func<float[], float> function, float[] probe_delta, float[] descent_k)
		{			
			parameters.CopyTo(changed_param1, 0);
			float fun_val = function(parameters);
			for (int i = 0; i < param_count; i++)
			{
				float old_param = parameters[i];
				if (Math.Abs(probe_delta[i]) < 1e-6f)
					probe_delta[i] = 1e-6f;
				changed_param1[i] = parameters[i] + probe_delta[i];
				float delta_value = function(changed_param1) - fun_val;
				partial_deriv[i] = delta_value / probe_delta[i];
				changed_param1[i] = old_param;
			}
			for (int i = 0; i < param_count; i++)
				parameters[i] = parameters[i] - descent_k[i] * partial_deriv[i];

			return parameters;
		}
	}


	class AdvancedGradientDescend
	{
		public AdvancedGradientDescend(int param_count, Func<float[], float> e_func)
		{
			this.param_count = param_count;
			error_func = e_func;
			param_vector = new float[3];
		}

		int param_count;
		Func<float[], float> error_func;
		
		public float[] param_vector;
		
		public float simple_gradient_error = 0.0f;
		public float advanced_gradient_error = 0.0f;
		public float 
	}
}
