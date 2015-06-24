function sqrerr = hybrid_sqerr(w_errors, batch_size, batch_weight)
    w_errors = [w_errors, batch_weight * sum(w_errors(1:batch_size))];
    sqrerr = meansqr(w_errors);
end

