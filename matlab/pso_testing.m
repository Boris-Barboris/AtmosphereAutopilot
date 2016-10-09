%sqr_func = @(x) sum(x.^2);
%[val, point] = PSO(sqr_func, [-2.0, -4.0; 3.0, 1.0], 10, 0.6, 0.5, 0.5, 100, true);
[val, point] = PSO(@aoa_objective_function, [0.0, 0.0; 10.0, 0.5], 10000, 0.6, 1.0, 1.0, 1, true);