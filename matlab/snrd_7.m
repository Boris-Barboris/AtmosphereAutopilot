function deriv = snrd_7(y,h)
%SNRD_7 Smooth noise robust diffirentiator
    lngth = length(y);
    deriv = zeros(1,lngth);
    for i = 4:lngth-3
        deriv(i) = (5 * (y(i+1) - y(i-1)) + 4 * (y(i+2) - y(i-2)) +...
            y(i+3) - y(i-3)) / (32 * h);
    end
end

