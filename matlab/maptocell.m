function index = maptocell(value, lower, upper, dims)
    diap = upper - lower;
    delta = diap ./ double(dims);
    index = ceil((value - lower) ./ delta);
    index = int16(clamp_index(index, dims));
end

function cindex = clamp_index(index, limits)
    cindex = index;
    for i = 1:length(index)
        if cindex(i) < 1
            cindex(i) = int16(1);
        elseif cindex(i) > limits(i)
            cindex(i) = limits(i);
        end
    end
end

