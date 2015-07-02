function index = coord2index(coord, dims)
    index = int16(coord2index_rec(coord, dims) + 1);
end

function index = coord2index_rec(coord, dims)
    dim_count = length(coord);
    if dim_count == 1
        index = coord;
    else
        dims = dims(2:end);
        temp = cumprod(dims);
        dim_step = temp(end);
        index = coord(1)*dim_step + coord2index_rec(coord(2:end), dims);
    end
end

