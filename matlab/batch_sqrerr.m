function sqrerr = batch_sqrerr(w_errors, batch_size, batch_count)
    bw_errors = zeros(1, size(w_errors,2) - (batch_size-1)*batch_count);
    for i = 1:batch_count
        bw_errors(i) = sum(w_errors((i-1)*batch_size+1 : i*batch_size));
        %bw_errors(i) = sum(w_errors(1 : i*batch_size));
    end
    bw_errors(i+1:end) = w_errors(i*batch_size+1 : end);
    sqrerr = meansqr(bw_errors);
end

