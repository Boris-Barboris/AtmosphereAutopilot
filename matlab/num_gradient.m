function gradient = num_gradient(f,x,h)
    gradient = arrayfun(@(dim)partial_deriv(f,x,dim,h(dim)),1:length(x));
end

function p_d = partial_deriv(f,x,dim,h)
    fx = f(x);
    x(dim) = x(dim) + h;
    fdx = f(x);
    p_d = (fdx - fx) ./ h;
end

