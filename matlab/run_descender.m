function mean_predict_error = run_descender(tick_limit, buffer_size, frame_size,...
    fixed_size, descend_k, miss_k, recent_factor, tolerance, state_space, smoothed_acc)

    series_length = length(state_space);

    % prepare performace analysis vectors
    ticks = int16(zeros(1,series_length));      % count error function evaluations
    error_values = zeros(1,series_length);      % error function final value on this slide
    param_values = zeros(4,series_length);      % regressed parameter vectors

    param_values(:,1) = [0,0,50,0].';
    constraints = [-Inf Inf; -Inf Inf; 0 Inf; -Inf 0];
    moi = 58;
    
    rng('default');

    % cycle on new input
    for frame_index = 1 : series_length
        % update frames
        frame = get_frame(state_space,buffer_size,frame_index,frame_size,fixed_size);
        % update error function
        e_func = @(x,frame)error_function(x(1),x(2),x(3),x(4),moi,...
            frame(1,:),frame(2,:),frame(3,:),frame(4,:),recent_factor);
        % do regression
        [param_values(:,frame_index), error_values(frame_index), ticks(frame_index)] =...
            frame_grad_descend(e_func, param_values(:,frame_index).', constraints, frame,...
            tick_limit, descend_k, miss_k, tolerance);
        if (frame_index < series_length)
            param_values(:,frame_index + 1) = param_values(:,frame_index);
        end
    end
    
    model_acc = angular_model(moi,param_values(1,:),...
        param_values(2,:), state_space(2,:), param_values(3,:),...
        state_space(3,:), param_values(4,:), state_space(4,:));
    
    model_predict_acc = [0.0, 0.0, angular_model(moi,param_values(1,1:series_length-2),...
        param_values(2,1:series_length-2),...
        state_space(2,3:series_length),param_values(3,1:series_length-2),...
        state_space(3,3:series_length),...
        param_values(4,1:series_length-2),state_space(4,3:series_length))];
    
    mean_predict_error = meansqr(model_predict_acc - smoothed_acc) +...
        meansqr(model_acc - smoothed_acc);
end

