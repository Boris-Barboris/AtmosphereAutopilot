function deriv = snrd_11(y,h)
%SNRD_11 Smooth noise robust diffirentiator
    lngth = length(y);
    deriv = zeros(1,lngth);
    for i = 6:lngth-5
        deriv(i) = (42 * (y(i+1) - y(i-1)) + 48 * (y(i+2) - y(i-2)) +...
            27 * (y(i+3) - y(i-3)) + 8 * (y(i+4) - y(i-4)) +...
            y(i+5) - y(i-5)) / (512 * h);
    end
end

