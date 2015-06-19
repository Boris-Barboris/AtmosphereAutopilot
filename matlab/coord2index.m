function index = coord2index(coord, dims)
    index = int16(coord2index_rec(coord, dims, 0));
end

function index = coord2index_rec(coord, dims, prev)
    dim_count = length(coord);
    if dim_count == 1
        index = prev + coord;
    else
        dims = dims(2:end);
        temp = cumprod(dims);
        dim_step = temp(end);
        index = coord2index_rec(coord(2:end), dims, prev + (coord(1)-1)*dim_step);
    end
end

