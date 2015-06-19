function deriv = snrd_9(y,h)
%SNRD_9 Smooth noise robust diffirentiator
    lngth = length(y);
    deriv = zeros(1,lngth);
    for i = 5:lngth-4
        deriv(i) = (14 * (y(i+1) - y(i-1)) + 14 * (y(i+2) - y(i-2)) +...
            6 * (y(i+3) - y(i-3)) + y(i+4) - y(i-4)) / (128 * h);
    end
end

