function [coord, cell_center] = maptocell(value, lower, upper, dims)
    span = upper - lower;
    cell_size = span ./ double(dims - 1);
    coord = floor((value - lower + (cell_size ./ 2.0)) ./ cell_size);
    coord = int16(clamp_index(coord, dims));
    cell_center = lower + double(coord) .* cell_size;
end

function cindex = clamp_index(index, limits)
    cindex = index;
    for i = 1:length(index)
        if cindex(i) < int16(0)
            cindex(i) = int16(0);
        elseif cindex(i) >= limits(i)
            cindex(i) = limits(i) - 1;
        end
    end
end

