function error = error_function(k_aoa, b_aoa, k_input, moi, acc, aoa, control, recent_factor)
    predicted = angular_model(moi, k_aoa, b_aoa, aoa, k_input,...
        control);
    importance = linspace(1.0,recent_factor,length(predicted));    
    diff = predicted - acc;
    error = sum((diff.*importance).^2);
end

