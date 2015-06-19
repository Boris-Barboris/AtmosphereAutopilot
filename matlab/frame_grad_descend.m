function [x_min, e_val, ticks] = frame_grad_descend(e_func, x, constraints, probe_delta, grad_mask,...
    frame, tick_limit, descend_k, misshot_k, tolerance)
    ticks = int16(0);
    x_min = x;
    e_val = 0;
    
    ignore_grad = false;
    grad = zeros(1, length(x));
    grad_sqrnorm = 1.0;
    
    while ticks < tick_limit
        if ~ignore_grad
            % get error in current state
            e_val = e_func(x_min, frame);
            % get error function gradient
            grad = num_gradient(@(t)e_func(t, frame),x_min,probe_delta) .* grad_mask;
            ticks = ticks + length(frame) + length(x)*length(frame);
            grad_sqrnorm = norm(grad)^2;
            steepness = norm(grad)/e_val;
            if steepness < tolerance
                return
            end
        end
        ignore_grad = false;
        % apply change for learning_speed
        learning_speed = e_val * (1 - descend_k) / grad_sqrnorm;
        % make descent
        x_min_new = x_min - grad .* learning_speed;
        for i = 1:length(x_min_new)
            if x_min_new(i) < constraints(i,1)
                x_min_new(i) = constraints(i,1);
            elseif x_min_new(i) > constraints(i,2)
                x_min_new(i) = constraints(i,2);
            end
        end
        e_val_new = e_func(x_min_new, frame);
        ticks = ticks + length(frame);
        % test and back off when we moved too much
        if e_val_new >= e_val
            % we've owershot, let's change descend_k
            descend_k = 1 - misshot_k * (1 - descend_k);
            ignore_grad = true;
        else
            x_min = x_min_new;
            e_val = e_val_new;
        end
    end
end

