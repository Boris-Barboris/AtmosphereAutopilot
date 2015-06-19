function acc = angular_model(moi, k_aoa, b_aoa, aoa, k_input, input)
    air_moment = k_aoa .* (aoa + b_aoa);
    input_moment = k_input .* input;
    if (abs(moi) < 1e-3)
        moi = 1e-3;
    end
    acc = (air_moment + input_moment) ./ moi;
end