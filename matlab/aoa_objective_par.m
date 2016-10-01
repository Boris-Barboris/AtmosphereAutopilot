function output_args = aoa_objective_par( input_args )
%AOA_OBJECTIVE_PAR Summary of this function goes here
%   Detailed explanation goes here
    results = zeros(20, 1);
    parfor i = 1:20
        results(i) = aoa_objective_function(input_args);
    end
    output_args = sum(results) / 20.0;
end

