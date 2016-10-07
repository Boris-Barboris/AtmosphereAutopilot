sqr_func = @(x) sum(x.^2);
[val, point] = PSO(sqr_func, [-2.0, -4.0; 3.0, 1.0], 10, 0.6, 0.5, 0.5, 20, true);