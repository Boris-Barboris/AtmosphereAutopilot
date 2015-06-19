function deriv = snrd_5(y,h)
%SNRD_5 Smooth noise robust diffirentiator
    lngth = length(y);
    deriv = zeros(1,lngth);
    for i = 3:lngth-2
        deriv(i) = (2 * (y(i+1) - y(i-1)) + y(i+2) - y(i-2)) / (8 * h);
    end
end

